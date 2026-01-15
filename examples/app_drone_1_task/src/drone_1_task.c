/*
// filepath: /home/austin/crazyflie-firmware/examples/app_drone_1_task/src/drone_1_task.c
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "DRONE1TASK"
#include "debug.h"

#include "log.h"
#include "led.h"
#include "commander.h"
#include "stabilizer_types.h"

#define AUX_ACTIVE_THRESH 1400
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
#define DIST2_CLOSE_MM 2000U // near-limit on distance2 (2m)
#define DIST2_HYST_MM 100U
#define ABORT_CONFIRM_COUNT 2 // Require N consecutive samples to trigger thresholds
#define AVOID_ENTER_CONFIRM_COUNT 2
#define AVOID_EXIT_CONFIRM_COUNT 20
#define AVOID_MIN_LAND_MM 500U
#define AVOID_SPEED_FACTOR 1.0f      // full speed during avoidance
#define AVOID_YAW_RATE_DPS 70.0f     // CW yaw rate for avoidance
#define DIST2_AVOID_MM DIST2_CLOSE_MM
#define DIST2_AVOID_HYST_MM DIST2_HYST_MM
#define AVOID_CONFIRM_COUNT ABORT_CONFIRM_COUNT
#define DEMO_TIME_MS 30000U // time of the demo in ms

static logVarId_t idAux0 = (logVarId_t)0xFFFF;
static logVarId_t idDistance0 = (logVarId_t)0xFFFF; // distance to the beacon
static logVarId_t idZ = (logVarId_t)0xFFFF;
static logVarId_t idYaw = (logVarId_t)0xFFFF;
static logVarId_t idDistance2 = (logVarId_t)0xFFFF; // distance to the other drone

static uint8_t abortOverCount = 0;
// New: inner bound enter/exit confirmation
static uint8_t innerOverCount = 0;
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

static inline bool aux0ActiveLow(void) {
  const int16_t v = logGetInt(idAux0);
  return (v > 0) && (v < AUX_ACTIVE_THRESH);
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

  // NOTE: distance2 no longer triggers emergency land; handled by avoidance mode
  return trigger;
}

// Decide avoidance activation based on distance2 only (no derivative)
static bool updateAvoidanceMode(void) {
  const uint32_t d2 = logGetUint(idDistance2);

  // Immediate land if too close during avoidance
  if (avoidActive && d2 > 0 && d2 <= AVOID_MIN_LAND_MM) {
    DEBUG_PRINT("AVOID EMERGENCY LAND: d2=%lu mm <= %u mm\n",
                (unsigned long)d2, AVOID_MIN_LAND_MM);
    seqAbort = true;
    return true;
  }

  if (!avoidActive) {
    if (d2 > 0 && d2 <= DIST2_AVOID_MM) {
      if (++approachCount >= AVOID_ENTER_CONFIRM_COUNT) {
        avoidActive = true;
        approachCount = 0;
        departCount = 0;
        DEBUG_PRINT("AVOID start: d2=%lu mm\n", (unsigned long)d2);
      }
    } else {
      approachCount = 0;
    }
  } else {
    if (d2 >= (DIST2_AVOID_MM + DIST2_AVOID_HYST_MM)) {
      if (++departCount >= AVOID_EXIT_CONFIRM_COUNT) {
        avoidActive = false;
        departCount = 0;
        approachCount = 0;
        DEBUG_PRINT("AVOID stop: d2=%lu mm\n", (unsigned long)d2);
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
  // - arcCooldown: once 270° arc is completed, do not re-trigger TURN until inner is re-entered
  bool arcActive = false;
  bool arcCooldown = false;
  float arcYawStart = 0.0f;
  float target_yaw = 0.0f;

  uint32_t startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

  DEBUG_PRINT("Reactive: fwd, TURN if d0>=%u; AVOID if d2<=%u, CW yaw; land if d0>=%u; stop after certain time\n",
              INNER_BOUND_MM, DIST2_AVOID_MM, DIST0_ABORT_MM);

  // Takeoff
  rampToHeight(TARGET_HEIGHT_M, RAMP_TIME_MS);

  enum { STRAIGHT = 0, TURN = 1 } mode = STRAIGHT;
  const uint32_t dtMs = 20;

  while (!seqAbort) {
    if (( xTaskGetTickCount() * portTICK_PERIOD_MS - startTime) > DEMO_TIME_MS) {
      break;
    }
    // Emergency checks (outer bound)
    if (checkAndMaybeEmergencyLand()) break;

    // Read distance0
    uint32_t d0 = logGetUint(idDistance0);

    // Clear arc cooldown once we re-enter the inner circle (with hysteresis)
    if (arcCooldown && d0 > 0 && d0 <= (INNER_BOUND_MM)) {
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
    if (seqAbort) break;
    if (avoid) {
      sendHover(FWD_SPEED_MPS * AVOID_SPEED_FACTOR, 0.0f, TARGET_HEIGHT_M, AVOID_YAW_RATE_DPS); // CW
    } else if (mode == STRAIGHT) {
      // STRAIGHT: constant forward velocity, zero yaw
      sendHover(FWD_SPEED_MPS, 0.0f, TARGET_HEIGHT_M, 0.0f);
    } else {
      // TURN: constant forward velocity, configured yaw rate
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
  DEBUG_PRINT("AUX0<%d triggers; inner=%u mm (turn), outer=%u mm (land), avoidance on d2<=%u (CW)\n",
              AUX_ACTIVE_THRESH, INNER_BOUND_MM, DIST0_ABORT_MM, DIST2_AVOID_MM);

  // Resolve required log IDs
  while (!logVarIdIsValid(idAux0) || 
    !logVarIdIsValid(idYaw)       ||
    !logVarIdIsValid(idDistance0) || 
    !logVarIdIsValid(idDistance2) ||
    !logVarIdIsValid(idZ) 
  ) {
    ensureLogId(&idAux0,      "cppm",    "aux0");
    ensureLogId(&idYaw, "stateEstimate", "yaw");
    ensureLogId(&idDistance0, "ranging", "distance0");
    ensureLogId(&idDistance2, "ranging", "distance2");
    ensureLogId(&idZ, "stateEstimate", "z");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ledSet(LED_BLUE_L, true);

  bool wasActive = false;
  while (1) {
    const bool active = aux0ActiveLow();

    // Emergency always active, even when idle
    if (checkAndMaybeEmergencyLand()) {
      ledSet(LED_BLUE_L, false);
      landToZero();
      ledSet(LED_BLUE_L, true);
    }

    if (active && !wasActive) {
      ledSet(LED_BLUE_L, false);
      runSequence();
      ledSet(LED_BLUE_L, true);
    }

    wasActive = active;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
