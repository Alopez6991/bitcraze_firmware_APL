/*
// filepath: /home/austin/crazyflie-firmware/examples/app_drone_2_task/src/drone_2_task.c
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "DRONE2TASK"
#include "debug.h"

#include "log.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"

// Variables and parameters for drone 2 task
#define AUX_ACTIVE_THRESHOLD 0U
#define TARGET_HEIGHT_M 1.0f
#define FWD_SPEED_MPS 0.5f
#define SEGMENT_TIME_MS 8000U
#define RAMP_TIME_MS 1500U
#define DIST0_ABORT_MM 3500U   // outer emergency bound (mm)
#define DIST0_HYST_MM 100U          // hysteresis margin
#define INNER_BOUND_MM 1750U // Inner bound to start turning
#define INNER_HYST_MM 100U
#define TURN_YAW_RATE_DPS 40.0f // Yaw rate while turning (deg/s)
#define LAND_VZ_MPS 0.4f            // descent speed
#define CUT_Z_M 0.05f               // cut controllers below this altitude
#define DIST1_CLOSE_MM 2000U // near-limit on distance1 (2m)
#define DIST1_HYST_MM 100U
#define ABORT_CONFIRM_COUNT 2 // Require N consecutive samples to trigger thresholds
#define AVOID_ENTER_CONFIRM_COUNT 2
#define AVOID_EXIT_CONFIRM_COUNT 10
#define AVOID_MIN_LAND_MM 500U
#define AVOID_SPEED_FACTOR 1.0f      // full speed during avoidance
#define AVOID_YAW_RATE_DPS -70.0f     // CCW yaw rate for avoidance
#define DIST1_AVOID_MM DIST1_CLOSE_MM
#define DIST1_AVOID_HYST_MM DIST1_HYST_MM
#define AVOID_CONFIRM_COUNT ABORT_CONFIRM_COUNT
#define DEMO_TIME_MS 30000U // time of the demo in ms


static logVarId_t idRangingAux1 = (logVarId_t)0xFFFF;
static logVarId_t idRangingAux2 = (logVarId_t)0xFFFF;
static logVarId_t idDistance0   = (logVarId_t)0xFFFF;
static logVarId_t idZ           = (logVarId_t)0xFFFF;
static logVarId_t idYaw = (logVarId_t)0xFFFF;
static logVarId_t idDistance1   = (logVarId_t)0xFFFF;

static uint8_t abortOverCount   = 0;
// Inner bound enter/exit confirmation
static uint8_t innerOverCount  = 0;
static uint8_t innerUnderCount = 0;

// Avoidance state
static bool avoidActive = false;
static uint8_t approachCount = 0;
static uint8_t departCount = 0;

// Sequence abort flag set by emergency check
static volatile bool seqAbort = false;

// Resolve a log var id if needed
static inline void ensureLogId(logVarId_t* id, const char* group, const char* name) {
  if (!logVarIdIsValid(*id)) {
    *id = logGetVarId(group, name);
  }
}

// Normalize angle to (-180, 180] deg
static inline float normalizeAngle(float a) {
  while (a <= -180) a += 360;
  while (a > 180) a -= 360;
  return a;
}

// Trigger: ranging.aux1 > 0
static inline bool sharedAux1Active(void) {
  const uint32_t v = logGetUint(idRangingAux1);
  return v > AUX_ACTIVE_THRESHOLD;
}

// Kill switch: ranging.aux2 > 0
static inline bool sharedAux2Active(void) {
  const uint32_t v = logGetUint(idRangingAux2);
  return v > AUX_ACTIVE_THRESHOLD;
}

static inline bool checkKillAndDisarm(void) { // check if this need to change
  if (sharedAux2Active()) {
    supervisorRequestArming(false); 
    commanderRelaxPriority(); // somehow this is necessary to ensure that it stays disarmed
    return true;
  }
  return false;
}

// Send a hover/vel setpoint (body frame XY, absolute Z)
static void sendHover(float vx, float vy, float z, float yawRateDeg) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));

  sp.mode.z = modeAbs;
  sp.position.z = z;

  sp.mode.x = modeVelocity;
  sp.mode.y = modeVelocity;
  sp.velocity_body = true;
  sp.velocity.x = vx;
  sp.velocity.y = vy;

  sp.mode.yaw = modeVelocity;
  sp.attitudeRate.yaw = yawRateDeg;

  commanderSetSetpoint(&sp, 3);
}

// static inline float getZ(void) { // not in task 1 ??
//   ensureLogId(&idZ, "stateEstimate", "z");
//   if (!logVarIdIsValid(idZ)) return -1.0f;
//   return logGetFloat(idZ);
// }

// Emergency checker: only outer bound on distance0
static bool checkAndMaybeEmergencyLand(void) {
  bool trigger = false;

  const uint32_t d0 = logGetUint(idDistance0);
  if (d0 > (DIST0_ABORT_MM + DIST0_HYST_MM)) {
    if (abortOverCount < 0xFF) abortOverCount++;
    if (abortOverCount >= ABORT_CONFIRM_COUNT) {
      if (!seqAbort) {
        DEBUG_PRINT("Emergency FAR: distance0=%lu mm (> %u+%u)\n",
                    (unsigned long)d0, DIST0_ABORT_MM, DIST0_HYST_MM);
      }
      seqAbort = true;
      trigger = true;
    }
  } else {
    abortOverCount = 0;
  }

  // NOTE: distance1 no longer triggers emergency land; handled by avoidance mode
  return trigger;
}

// Decide avoidance activation based on distance1 only (no derivative)
static bool updateAvoidanceMode(void) {
    const uint32_t d1 = logGetUint(idDistance1);

  // If already in avoidance and we get dangerously close, land immediately
  if (avoidActive && d1 > 0 && d1 <= AVOID_MIN_LAND_MM) {
    DEBUG_PRINT("AVOID EMERGENCY LAND: d1=%lu mm <= %u mm\n",
                (unsigned long)d1, AVOID_MIN_LAND_MM);
    seqAbort = true;
    return true;  // let caller command one more safe tick; loop will exit
  }

  if (!avoidActive) {
    if (d1 > 0 && d1 <= DIST1_AVOID_MM) {
      if (++approachCount >= AVOID_ENTER_CONFIRM_COUNT) {
        avoidActive = true;
        approachCount = 0;
        departCount = 0;
        DEBUG_PRINT("AVOID start: d1=%lu mm\n", (unsigned long)d1);
      }
    } else {
      approachCount = 0;
    }
  } else {
    // Strong exit: above threshold + hysteresis for many samples
    if (d1 >= (DIST1_AVOID_MM + DIST1_AVOID_HYST_MM)) {
      if (++departCount >= AVOID_EXIT_CONFIRM_COUNT) {
        avoidActive = false;
        departCount = 0;
        approachCount = 0;
        DEBUG_PRINT("AVOID stop: d1=%lu mm\n", (unsigned long)d1);
      }
    } else {
      departCount = 0;
    }
  }

  return avoidActive;
}

static void landToZero(void) {
  // Descend at constant vertical velocity until near ground
  setpoint_t sp; memset(&sp, 0, sizeof(sp));
  sp.mode.x = modeVelocity; sp.velocity.x = 0;
  sp.mode.y = modeVelocity; sp.velocity.y = 0;
  sp.mode.z = modeVelocity; sp.velocity.z = -LAND_VZ_MPS;
  sp.mode.yaw = modeVelocity; sp.attitudeRate.yaw = 0;
  sp.velocity_body = true;
  while (1) {
    float z = logGetFloat(idZ);
    if (z >= 0.0f && z <= CUT_Z_M) break; // landing altitude reached
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  // Cut controllers and thrust to avoid bouncing
  setpoint_t cut; memset(&cut, 0, sizeof(cut));
  cut.mode.x = modeDisable;
  cut.mode.y = modeDisable;
  cut.mode.z = modeDisable;
  cut.mode.yaw = modeDisable;
  cut.thrust = 0;
  for (int i = 0; i < 50; i++) {
    commanderSetSetpoint(&cut, 3);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void rampToHeight(float zTarget, uint32_t rampMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = (rampMs / dtMs) ? (rampMs / dtMs) : 1;
  for (uint32_t i = 0; i <= steps; i++) {
    if (checkKillAndDisarm()) return;
    if (checkAndMaybeEmergencyLand()) return;
    const float z = (zTarget * (float)i) / (float)steps;
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}


static void runSequence(void) {
  seqAbort = false;
  innerOverCount = innerUnderCount = 0;
  uint8_t routines = 0;  // count of (exit inner bound -> re-enter) cycles
  // Arc tracking/cooldown:
  // - arcActive: we are tracking rotation since inner exit while in TURN
  // - arcCooldown: once the 270° arc is completed, do not re-trigger TURN until inner is re-entered
  bool arcActive = false;
  bool arcCooldown = false;
  float arcYawStart = 0.0f;
  float target_yaw = 0.0f;

  uint32_t startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

  DEBUG_PRINT("Reactive: fwd, TURN if d0>=%u; AVOID if d1<=%u, CCW yaw; land if d0>=%u; stop after 2 routines\n",
              INNER_BOUND_MM, DIST1_AVOID_MM, DIST0_ABORT_MM);

  // Kill before takeoff
  if (checkKillAndDisarm()) return;

  // Takeoff
  rampToHeight(TARGET_HEIGHT_M, RAMP_TIME_MS);

  enum { STRAIGHT = 0, TURN = 1 } mode = STRAIGHT;
  const uint32_t dtMs = 20;

  while (!seqAbort) {
    if (checkKillAndDisarm()) return;
    if (( xTaskGetTickCount() * portTICK_PERIOD_MS - startTime) > DEMO_TIME_MS) {
      break;
    }
    // Emergency checks (outer bound)
    if (checkAndMaybeEmergencyLand()) break;

    // Read distance0
    const uint32_t d0 = logGetUint(idDistance0);

    // Clear arc cooldown once we re-enter the inner circle (no hysteresis change here)
    if (arcCooldown && d0 > 0 && d0 <= INNER_BOUND_MM) {
      arcCooldown = false;
      DEBUG_PRINT("ARC cooldown cleared by inner re-entry (d0=%lu)\n", (unsigned long)d0);
    }

    // Mode transitions with hysteresis + confirmation
    if (mode == STRAIGHT) {
      if (d0 >= (INNER_BOUND_MM)) {
        if (!arcCooldown && (++innerOverCount >= ABORT_CONFIRM_COUNT)) {
          // Exiting inner radius: enter TURN and start heading tracking
          mode = TURN;
          innerOverCount = 0;
          arcYawStart = logGetFloat(idYaw);
          target_yaw = normalizeAngle(arcYawStart - 90.0f); // 270 (-90) degrees from start
          arcActive = isfinite(arcYawStart);
          DEBUG_PRINT("ENTER TURN (d0=%lu), arcActive=%d, startYaw=%.3f deg\n",
                      (unsigned long)d0, arcActive ? 1 : 0, (double)arcYawStart);
        }
      } else {
        innerOverCount = 0;
      }
    } else { // TURN
      if (d0 <= (INNER_BOUND_MM)) {
        if (++innerUnderCount >= ABORT_CONFIRM_COUNT) {
          mode = STRAIGHT;
          innerUnderCount = 0;
          routines++;
          DEBUG_PRINT("EXIT TURN -> routine %u complete (d0=%lu)\n", routines, (unsigned long)d0);
        }
      } else {
        innerUnderCount = 0;
      }
    }

    // While TURN is active, accumulate rotation relative to arcYawStart.
    // When 270° reached (in the configured yaw direction), stop yaw by switching to STRAIGHT.
    if (mode == TURN && arcActive) {
      float curYaw = logGetFloat(idYaw);
      if (isfinite(curYaw)) {
        const bool reached = (fabsf(curYaw - target_yaw) <= 3.0f) || (fabsf(curYaw - target_yaw - 360.0f) <= 3.0f);
        if (reached) {
          DEBUG_PRINT("TURN arc reached 270° -> stop yaw\n");
          // Stop yaw by switching to STRAIGHT; keep forward velocity
          mode = STRAIGHT;
          arcActive = false;
          arcCooldown = true;  // do not re-trigger TURN until inner is re-entered
          innerUnderCount = 0; // avoid instant STRAIGHT->TURN flip-flop
        }
      } else {
        // Lost yaw; stop arc tracking to avoid undefined behavior
        arcActive = false;
        DEBUG_PRINT("TURN arc: yaw unavailable, stopping arc tracking\n");
      }
    }

    // Avoidance overrides the normal command
    const bool avoid = updateAvoidanceMode();
    if (seqAbort) break;  // triggered by minimum distance during avoidance
    if (avoid) {
      sendHover(FWD_SPEED_MPS * AVOID_SPEED_FACTOR, 0.0f, TARGET_HEIGHT_M, AVOID_YAW_RATE_DPS);
    } else if (mode == STRAIGHT) {
      sendHover(FWD_SPEED_MPS, 0.0f, TARGET_HEIGHT_M, 0.0f);
    } else {
      sendHover(FWD_SPEED_MPS, 0.0f, TARGET_HEIGHT_M, TURN_YAW_RATE_DPS);
    }

    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }

    landToZero();
  if (seqAbort) {
    DEBUG_PRINT("Emergency landing\n");
  } else {
    landToZero();
    DEBUG_PRINT("Finished the demo in approx %u routines)\n", routines);
  }
}

void appMain(void) {
  DEBUG_PRINT("drone_2_task: ranging.aux1 triggers; inner=%u mm (turn), outer=%u mm (land), avoidance on d1<=%u (CCW)\n",
              INNER_BOUND_MM, DIST0_ABORT_MM, DIST1_AVOID_MM);

  // Resolve required log IDs (include aux2 for kill)
  while (!logVarIdIsValid(idRangingAux1) || !logVarIdIsValid(idRangingAux2) ||
         !logVarIdIsValid(idDistance0) || !logVarIdIsValid(idDistance1) ||
         !logVarIdIsValid(idZ) || !logVarIdIsValid(idYaw)) {

    ensureLogId(&idRangingAux1, "ranging", "aux1");
    ensureLogId(&idRangingAux2, "ranging", "aux2");
    ensureLogId(&idDistance0,   "ranging", "distance0");
    ensureLogId(&idDistance1,   "ranging", "distance1");
    ensureLogId(&idZ,           "stateEstimate", "z");
    ensureLogId(&idYaw,         "stateEstimate", "yaw");

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  bool wasActive = false;
  while (1) {
    const bool active = sharedAux1Active();

    // Emergency always active, even when idle
    if (checkAndMaybeEmergencyLand()) {
      landToZero();
    }

    // On rising edge of trigger, run the sequence
    if (active && !wasActive) {
      runSequence();
    }

    wasActive = active;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
