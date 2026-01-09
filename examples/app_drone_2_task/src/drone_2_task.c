/*
// filepath: /home/austin/crazyflie-firmware/examples/app_drone_2_task/src/drone_2_task.c
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "DRONE2TASK"
#include "debug.h"

#include "log.h"
#include "led.h"
#include "commander.h"
#include "stabilizer_types.h"

#ifndef TARGET_HEIGHT_M
#define TARGET_HEIGHT_M 0.8f
#endif

#ifndef FWD_SPEED_MPS
#define FWD_SPEED_MPS 0.5f
#endif

#ifndef SEGMENT_TIME_MS
#define SEGMENT_TIME_MS 8000U
#endif

#ifndef RAMP_TIME_MS
#define RAMP_TIME_MS 1500U
#endif

#ifndef DIST0_ABORT_MM
#define DIST0_ABORT_MM 3500U   // outer emergency bound (mm)
#endif
#ifndef DIST0_HYST_MM
#define DIST0_HYST_MM 100U          // hysteresis margin
#endif
#ifndef ABORT_CONFIRM_COUNT
#define ABORT_CONFIRM_COUNT 5
#endif
// Stronger exit requirement: many consecutive "departing" samples
#ifndef AVOID_ENTER_CONFIRM_COUNT
#define AVOID_ENTER_CONFIRM_COUNT 2                     // very few samples to enter
#endif
#ifndef AVOID_EXIT_CONFIRM_COUNT
#define AVOID_EXIT_CONFIRM_COUNT 20                     // stop avoidance after sustained departing
#endif
// Minimum distance during avoidance: land if closer than 0.5 m
#ifndef AVOID_MIN_LAND_MM
#define AVOID_MIN_LAND_MM 500U
#endif

// --- Avoidance parameters (drone 2 = CCW yaw) ---
#ifndef AVOID_SPEED_FACTOR
#define AVOID_SPEED_FACTOR 1.0f      // half speed during avoidance
#endif
#ifndef AVOID_YAW_RATE_DPS
#define AVOID_YAW_RATE_DPS -70.0f    // CCW yaw rate for avoidance
#endif
#ifndef DIST1_AVOID_MM
#define DIST1_AVOID_MM DIST1_CLOSE_MM
#endif
#ifndef DIST1_AVOID_HYST_MM
#define DIST1_AVOID_HYST_MM DIST1_HYST_MM
#endif
#ifndef AVOID_CONFIRM_COUNT
#define AVOID_CONFIRM_COUNT ABORT_CONFIRM_COUNT
#endif

#ifndef LAND_VZ_MPS
#define LAND_VZ_MPS 0.4f            // descent speed
#endif
#ifndef CUT_Z_M
#define CUT_Z_M 0.05f               // cut controllers below this altitude
#endif
// Abort if distance1 is closer than 1 m (with hysteresis)
#ifndef DIST1_CLOSE_MM
#define DIST1_CLOSE_MM 2000U
#endif
#ifndef DIST1_HYST_MM
#define DIST1_HYST_MM 100U
#endif
// Inner bound to start turning
#ifndef INNER_BOUND_MM
#define INNER_BOUND_MM 1750U
#endif
#ifndef INNER_HYST_MM
#define INNER_HYST_MM 100U
#endif
// Yaw rate while turning (deg/s)
#ifndef TURN_YAW_RATE_DPS
#define TURN_YAW_RATE_DPS 40.0f
#endif

static logVarId_t idRangingAux1 = (logVarId_t)0xFFFF;
static logVarId_t idRangingAux2 = (logVarId_t)0xFFFF;
static logVarId_t idDistance0   = (logVarId_t)0xFFFF;
static logVarId_t idZ           = (logVarId_t)0xFFFF;
// Add: distance1
static logVarId_t idDistance1   = (logVarId_t)0xFFFF;

static uint8_t abortOverCount   = 0;
// Remove near-limit land counter usage; keep var if needed elsewhere
static uint8_t closeUnderCount __attribute__((unused)) = 0;
// Inner bound enter/exit confirmation
static uint8_t innerOverCount  = 0;
static uint8_t innerUnderCount = 0;

// Avoidance state
static bool avoidActive = false;
static uint32_t lastD1 __attribute__((unused)) = 0;
static bool lastD1Valid __attribute__((unused)) = false;
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

// Trigger: ranging.aux1 > 0
static inline bool sharedAux1Active(void) {
  ensureLogId(&idRangingAux1, "ranging", "aux1");
  if (!logVarIdIsValid(idRangingAux1)) return false;
  const uint32_t v = logGetUint(idRangingAux1);
  return v > 0;
}

// Kill switch: ranging.aux2 > 0
static inline bool sharedAux2Active(void) {
  ensureLogId(&idRangingAux2, "ranging", "aux2");
  if (!logVarIdIsValid(idRangingAux2)) return false;
  const uint32_t v = logGetUint(idRangingAux2);
  return v > 0;
}

// Immediate disarm: cut all controllers and thrust
static void killDisarm(void) {
  setpoint_t cut; memset(&cut, 0, sizeof(cut));
  cut.mode.x = modeDisable;
  cut.mode.y = modeDisable;
  cut.mode.z = modeDisable;
  cut.mode.yaw = modeDisable;
  cut.thrust = 0;
  for (int i = 0; i < 100; i++) {          // ~1 s to ensure radio gets it
    commanderSetSetpoint(&cut, 3);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  seqAbort = true;
  DEBUG_PRINT("KILL: AUX2 active -> disarm\n");
}

static inline bool checkKillAndDisarm(void) {
  if (sharedAux2Active()) { killDisarm(); return true; }
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

static inline float getZ(void) {
  ensureLogId(&idZ, "stateEstimate", "z");
  if (!logVarIdIsValid(idZ)) return -1.0f;
  return logGetFloat(idZ);
}

// Emergency checker: only outer bound on distance0
static bool checkAndMaybeEmergencyLand(void) {
  ensureLogId(&idDistance0, "ranging", "distance0");
  bool trigger = false;

  if (logVarIdIsValid(idDistance0)) {
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
  }

  // NOTE: distance1 no longer triggers emergency land; handled by avoidance mode
  return trigger;
}

// Decide avoidance activation based on distance1 only (no derivative)
static bool updateAvoidanceMode(void) {
  ensureLogId(&idDistance1, "ranging", "distance1");
  if (!logVarIdIsValid(idDistance1)) {
    return false;
  }

  const uint32_t d1 = logGetUint(idDistance1);

  // If already in avoidance and we get dangerously close, land immediately
  if (avoidActive && d1 > 0 && d1 <= AVOID_MIN_LAND_MM) {
    DEBUG_PRINT("AVOID EMERGENCY LAND: d1=%lu mm <= %u mm\n",
                (unsigned long)d1, AVOID_MIN_LAND_MM);
    seqAbort = true;
    return true;  // let caller command one more safe tick; loop will exit
  }

  if (!avoidActive) {
    // Liberal entry: inside threshold for a very few samples
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

static void landEmergency(void) {
  // Descend at constant vertical velocity until near ground
  while (1) {
    float z = getZ();
    if (z >= 0.0f && z <= CUT_Z_M) break;

    setpoint_t sp; memset(&sp, 0, sizeof(sp));
    sp.mode.x = modeVelocity; sp.velocity.x = 0;
    sp.mode.y = modeVelocity; sp.velocity.y = 0;
    sp.mode.z = modeVelocity; sp.velocity.z = -LAND_VZ_MPS;
    sp.mode.yaw = modeVelocity; sp.attitudeRate.yaw = 0;
    sp.velocity_body = true;
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

static void __attribute__((unused)) holdAtHeight(float z, uint32_t holdMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = holdMs / dtMs;
  for (uint32_t i = 0; i < steps; i++) {
    if (checkAndMaybeEmergencyLand()) return;
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

static void __attribute__((unused)) flyBodyVX(float vx, float z, uint32_t durationMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = durationMs / dtMs;
  for (uint32_t i = 0; i < steps; i++) {
    if (checkAndMaybeEmergencyLand()) return;
    sendHover(vx, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

static void landToZero(uint32_t rampMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = (rampMs / dtMs) ? (rampMs / dtMs) : 1;
  for (uint32_t i = 0; i <= steps; i++) {
    const float z = TARGET_HEIGHT_M * (1.0f - (float)i / (float)steps);
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
  for (int i = 0; i < 20; i++) {
    sendHover(0.0f, 0.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void runSequence(void) {
  seqAbort = false;
  innerOverCount = innerUnderCount = 0;
  uint8_t routines = 0;  // count of (exit inner bound -> re-enter) cycles

  DEBUG_PRINT("Reactive: fwd, TURN if d0>=%u; AVOID if d1<=%u, CCW yaw; land if d0>=%u; stop after 2 routines\n",
              INNER_BOUND_MM, DIST1_AVOID_MM, DIST0_ABORT_MM);

  // Kill before takeoff
  if (checkKillAndDisarm()) return;

  // Takeoff
  rampToHeight(TARGET_HEIGHT_M, RAMP_TIME_MS);
  if (seqAbort) { landEmergency(); return; }

  enum { STRAIGHT = 0, TURN = 1 } mode = STRAIGHT;
  const uint32_t dtMs = 20;

  while (!seqAbort && routines < 5) {
    // Kill supersedes everything
    if (checkKillAndDisarm()) break;

    // Emergency checks (outer bound)
    if (checkAndMaybeEmergencyLand()) break;

    // Read distance0
    ensureLogId(&idDistance0, "ranging", "distance0");
    const uint32_t d0 = logVarIdIsValid(idDistance0) ? logGetUint(idDistance0) : 0;

    // Mode transitions with hysteresis + confirmation
    if (mode == STRAIGHT) {
      if (d0 >= (INNER_BOUND_MM + INNER_HYST_MM)) {
        if (++innerOverCount >= ABORT_CONFIRM_COUNT) {
          mode = TURN;
          innerOverCount = 0;
          DEBUG_PRINT("ENTER TURN (d0=%lu)\n", (unsigned long)d0);
        }
      } else {
        innerOverCount = 0;
      }
    } else { // TURN
      if (d0 <= (INNER_BOUND_MM - INNER_HYST_MM)) {
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

  if (seqAbort) {
    landEmergency();
  } else {
    landToZero(RAMP_TIME_MS);
    DEBUG_PRINT("Done (routines=%u)\n", routines);
  }
}

void appMain(void) {
  DEBUG_PRINT("drone_2_task: ranging.aux1 triggers; inner=%u mm (turn), outer=%u mm (land), avoidance on d1<=%u (CCW)\n",
              INNER_BOUND_MM, DIST0_ABORT_MM, DIST1_AVOID_MM);

  // Resolve required log IDs (include aux2 for kill)
  while (!logVarIdIsValid(idRangingAux1) || !logVarIdIsValid(idRangingAux2) ||
         !logVarIdIsValid(idDistance0) || !logVarIdIsValid(idDistance1)) {
    ensureLogId(&idRangingAux1, "ranging", "aux1");
    ensureLogId(&idRangingAux2, "ranging", "aux2");
    ensureLogId(&idDistance0,   "ranging", "distance0");
    ensureLogId(&idDistance1,   "ranging", "distance1");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ledSet(LED_BLUE_L, true);

  bool wasActive = false;
  while (1) {
    // KILL: disarm immediately, supersedes everything
    if (sharedAux2Active()) {
      ledSet(LED_BLUE_L, false);
      killDisarm();
      // stay disarmed while aux2 is held
      while (sharedAux2Active()) { vTaskDelay(pdMS_TO_TICKS(20)); }
      ledSet(LED_BLUE_L, true);
    }

    const bool active = sharedAux1Active();

    // Emergency always active, even when idle
    if (checkAndMaybeEmergencyLand()) {
      ledSet(LED_BLUE_L, false);
      landEmergency();
      ledSet(LED_BLUE_L, true);
    }

    // On rising edge of trigger, run the sequence
    if (active && !wasActive) {
      ledSet(LED_BLUE_L, false);
      runSequence();
      ledSet(LED_BLUE_L, true);
    }

    wasActive = active;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
