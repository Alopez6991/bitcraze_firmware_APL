/*
// filepath: /home/austin/crazyflie-firmware/examples/app_drone_1_task/src/drone_1_task.c
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "DRONE1TASK"
#include "debug.h"

#include "log.h"
#include "led.h"
#include "commander.h"
#include "stabilizer_types.h"

#ifndef AUX_ACTIVE_THRESH
#define AUX_ACTIVE_THRESH 1400
#endif

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
#ifndef LAND_VZ_MPS
#define LAND_VZ_MPS 0.4f            // descent speed
#endif
#ifndef CUT_Z_M
#define CUT_Z_M 0.05f               // cut controllers below this altitude
#endif
// New: near-limit on distance2 (1 m)
#ifndef DIST2_CLOSE_MM
#define DIST2_CLOSE_MM 2000U
#endif
#ifndef DIST2_HYST_MM
#define DIST2_HYST_MM 100U
#endif
// Obstacle (up sensor) emergency thresholds
#ifndef OBST_UP_THRESH_MM
#define OBST_UP_THRESH_MM 750U      // trigger if up < 0.75 m
#endif
#ifndef OBST_HYST_MM
#define OBST_HYST_MM 50U             // hysteresis margin
#endif
#ifndef OBST_CONFIRM_COUNT
#define OBST_CONFIRM_COUNT 3         // samples required to confirm obstacle
#endif

// Obstacle response maneuver parameters
#ifndef OBST_BACK_VX_MPS
#define OBST_BACK_VX_MPS -0.5f      // fly backwards at 0.5 m/s (body X negative)
#endif
#ifndef OBST_BACKOFF_MS
#define OBST_BACKOFF_MS 2000U       // 2 seconds back off
#endif
#ifndef OBST_TURN_YAW_RATE_DPS
#define OBST_TURN_YAW_RATE_DPS 30.0f // turn in place at 30 deg/s
#endif
#ifndef OBST_TURN_MS
#define OBST_TURN_MS 2000U          // 2 seconds turn
#endif

// Require N consecutive samples to trigger thresholds
#ifndef ABORT_CONFIRM_COUNT
#define ABORT_CONFIRM_COUNT 5
#endif
#ifndef AVOID_ENTER_CONFIRM_COUNT
#define AVOID_ENTER_CONFIRM_COUNT 2
#endif
#ifndef AVOID_EXIT_CONFIRM_COUNT
#define AVOID_EXIT_CONFIRM_COUNT 20
#endif
#ifndef AVOID_MIN_LAND_MM
#define AVOID_MIN_LAND_MM 500U
#endif
#ifndef AVOID_SPEED_FACTOR
#define AVOID_SPEED_FACTOR 1.0f      // full speed during avoidance
#endif
// --- Avoidance parameters (drone 1 = CW yaw) ---
#ifndef AVOID_YAW_RATE_DPS
#define AVOID_YAW_RATE_DPS 70.0f     // CW yaw rate for avoidance
#endif
#ifndef DIST2_AVOID_MM
#define DIST2_AVOID_MM DIST2_CLOSE_MM
#endif
#ifndef DIST2_AVOID_HYST_MM
#define DIST2_AVOID_HYST_MM DIST2_HYST_MM
#endif
#ifndef AVOID_CONFIRM_COUNT
#define AVOID_CONFIRM_COUNT ABORT_CONFIRM_COUNT
#endif

static logVarId_t idAux0 = (logVarId_t)0xFFFF;
static logVarId_t idDistance0 = (logVarId_t)0xFFFF;
static logVarId_t idZ = (logVarId_t)0xFFFF;
// New: distance2
static logVarId_t idDistance2 = (logVarId_t)0xFFFF;
// New: multiranger up sensor
static logVarId_t idUp = (logVarId_t)0xFFFF;

static uint8_t abortOverCount = 0;
// New: counter for near-limit
static uint8_t closeUnderCount __attribute__((unused)) = 0;
// New: inner bound enter/exit confirmation
static uint8_t innerOverCount = 0;
static uint8_t innerUnderCount = 0;
// New: obstacle confirmation counter
static uint8_t obstUnderCount = 0;
static bool obstLatched = false;                      // latch until cleared by hysteresis
static volatile bool obstacleActionRequested = false; // set when we should execute the maneuver

// Avoidance state
static bool avoidActive = false;
static uint32_t lastD2 __attribute__((unused)) = 0;
static bool lastD2Valid __attribute__((unused)) = false;
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
// Resolve 'up' from multiranger, with fallback to 'range'
static inline void ensureUpLogId(void) {
  if (!logVarIdIsValid(idUp)) {
    idUp = logGetVarId("multiranger", "up");
    if (!logVarIdIsValid(idUp)) {
      idUp = logGetVarId("range", "up");
    }
  }
}

static inline bool aux0ActiveLow(void) {
  ensureLogId(&idAux0, "cppm", "aux0");
  if (!logVarIdIsValid(idAux0)) return false;
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

static inline float getZ(void) {
  ensureLogId(&idZ, "stateEstimate", "z");
  if (!logVarIdIsValid(idZ)) return -1.0f;
  return logGetFloat(idZ);
}

static bool checkAndMaybeEmergencyLand(void) {
  ensureLogId(&idDistance0, "ranging", "distance0");
  ensureLogId(&idDistance2, "ranging", "distance2");
  ensureUpLogId();

  bool trigger = false;

  // Outer bound emergency on distance0
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

  // Obstacle on Multiranger 'up': request maneuver (back off + turn) instead of landing
  if (logVarIdIsValid(idUp)) {
    const uint32_t up = logGetUint(idUp);
    if (up > 0 && up <= OBST_UP_THRESH_MM) {
      if (!obstLatched) {
        if (obstUnderCount < 0xFF) obstUnderCount++;
        if (obstUnderCount >= OBST_CONFIRM_COUNT) {
          obstLatched = true;                // latch until cleared by hysteresis
          obstacleActionRequested = true;    // ask the main loop to execute the maneuver
          DEBUG_PRINT("OBST UP: %lu mm (<= %u) -> maneuver requested\n",
                      (unsigned long)up, OBST_UP_THRESH_MM);
        }
      }
    } else if (up >= (OBST_UP_THRESH_MM + OBST_HYST_MM) || up == 0) {
      // Clear latch and counter when safely out of the band or invalid
      obstUnderCount = 0;
      obstLatched = false;
    }
  }

  // NOTE: distance2 handled by avoidance mode (not emergency)
  return trigger;
}

// Decide avoidance activation based on distance2 only (no derivative)
static bool updateAvoidanceMode(void) {
  ensureLogId(&idDistance2, "ranging", "distance2");
  if (!logVarIdIsValid(idDistance2)) return false;

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

// Execute the obstacle response: back off for 2 s, then turn in place for 2 s
static void performObstacleManeuver(void) {
  const uint32_t dtMs = 20;
  uint32_t steps = OBST_BACKOFF_MS / dtMs;
  DEBUG_PRINT("OBST MANEUVER: back off for %u ms at %.2f m/s\n", OBST_BACKOFF_MS, (double)OBST_BACK_VX_MPS);
  for (uint32_t i = 0; i < steps; i++) {
    if (checkAndMaybeEmergencyLand()) return; // outer bound can still abort
    sendHover(OBST_BACK_VX_MPS, 0.0f, TARGET_HEIGHT_M, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
  DEBUG_PRINT("OBST MANEUVER: turn in place for %u ms at %.1f deg/s\n", OBST_TURN_MS, (double)OBST_TURN_YAW_RATE_DPS);
  steps = OBST_TURN_MS / dtMs;
  for (uint32_t i = 0; i < steps; i++) {
    if (checkAndMaybeEmergencyLand()) return;
    sendHover(0.0f, 0.0f, TARGET_HEIGHT_M, OBST_TURN_YAW_RATE_DPS);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

static void runSequence(void) {
  seqAbort = false;
  innerOverCount = innerUnderCount = 0;
  uint8_t routines = 0;  // count of (exit inner bound -> re-enter) cycles

  DEBUG_PRINT("Reactive: fwd, TURN if d0>=%u; AVOID if d2<=%u, CW yaw; OBST up<%u: back+turn; land if d0>=%u; stop after 2 routines\n",
              INNER_BOUND_MM, DIST2_AVOID_MM, OBST_UP_THRESH_MM, DIST0_ABORT_MM);

  // Takeoff
  rampToHeight(TARGET_HEIGHT_M, RAMP_TIME_MS);
  if (seqAbort) { landEmergency(); return; }

  enum { STRAIGHT = 0, TURN = 1 } mode = STRAIGHT;
  const uint32_t dtMs = 20;

  while (!seqAbort && routines < 5) {
    // Emergency checks (outer bound)
    if (checkAndMaybeEmergencyLand()) break;

    // Obstacle action supersedes normal/avoidance commands
    if (obstacleActionRequested) {
      obstacleActionRequested = false; // consume the request
      performObstacleManeuver();
      // after maneuver, continue the loop
      continue;
    }

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
    if (seqAbort) break;
    if (avoid) {
      sendHover(FWD_SPEED_MPS * AVOID_SPEED_FACTOR, 0.0f, TARGET_HEIGHT_M, AVOID_YAW_RATE_DPS); // CW
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
  DEBUG_PRINT("AUX0<%d triggers; inner=%u mm (turn), outer=%u mm (land), avoidance on d2<=%u (CW), obstacle up<%u mm (land)\n",
              AUX_ACTIVE_THRESH, INNER_BOUND_MM, DIST0_ABORT_MM, DIST2_AVOID_MM, OBST_UP_THRESH_MM);
  // Resolve required log IDs
  while (!logVarIdIsValid(idAux0) || !logVarIdIsValid(idDistance0) || !logVarIdIsValid(idDistance2)) {
    ensureLogId(&idAux0,      "cppm",    "aux0");
    ensureLogId(&idDistance0, "ranging", "distance0");
    ensureLogId(&idDistance2, "ranging", "distance2");
    ensureUpLogId();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ledSet(LED_BLUE_L, true);

  bool wasActive = false;
  while (1) {
    const bool active = aux0ActiveLow();

    // Emergency always active, even when idle
    if (checkAndMaybeEmergencyLand()) {
      ledSet(LED_BLUE_L, false);
      landEmergency();
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
