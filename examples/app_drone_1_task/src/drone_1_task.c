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
#define DIST0_ABORT_MM 2000U
#endif
#ifndef DIST0_HYST_MM
#define DIST0_HYST_MM 100U          // hysteresis margin
#endif
#ifndef ABORT_CONFIRM_COUNT
#define ABORT_CONFIRM_COUNT 5       // require N consecutive samples
#endif
#ifndef LAND_VZ_MPS
#define LAND_VZ_MPS 0.4f            // descent speed
#endif
#ifndef CUT_Z_M
#define CUT_Z_M 0.05f               // cut controllers below this altitude
#endif

static logVarId_t idAux0 = (logVarId_t)0xFFFF;
static logVarId_t idDistance0 = (logVarId_t)0xFFFF;
static logVarId_t idZ = (logVarId_t)0xFFFF;
static uint8_t abortOverCount = 0;

// Sequence abort flag set by emergency check
static volatile bool seqAbort = false;

// Resolve a log var id if needed
static inline void ensureLogId(logVarId_t* id, const char* group, const char* name) {
  if (!logVarIdIsValid(*id)) {
    *id = logGetVarId(group, name);
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
  if (!logVarIdIsValid(idDistance0)) return false;

  const uint32_t d0 = logGetUint(idDistance0);
  if (d0 > (DIST0_ABORT_MM + DIST0_HYST_MM)) {
    if (abortOverCount < 0xFF) abortOverCount++;
    if (abortOverCount >= ABORT_CONFIRM_COUNT) {
      if (!seqAbort) {
        DEBUG_PRINT("Emergency: distance0=%lu mm (> %u+%u) -> landing\n",
                    (unsigned long)d0, DIST0_ABORT_MM, DIST0_HYST_MM);
      }
      seqAbort = true;
      return true;
    }
  } else {
    abortOverCount = 0;
  }
  return false;
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

static void holdAtHeight(float z, uint32_t holdMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = holdMs / dtMs;
  for (uint32_t i = 0; i < steps; i++) {
    if (checkAndMaybeEmergencyLand()) return;
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

static void flyBodyVX(float vx, float z, uint32_t durationMs) {
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
  DEBUG_PRINT("Sequence: takeoff -> fwd -> stop -> back -> land (abort with hysteresis)\n");

  rampToHeight(TARGET_HEIGHT_M, RAMP_TIME_MS);        if (seqAbort) { landEmergency(); return; }
  holdAtHeight(TARGET_HEIGHT_M, 300);                  if (seqAbort) { landEmergency(); return; }

  flyBodyVX(FWD_SPEED_MPS, TARGET_HEIGHT_M, SEGMENT_TIME_MS);
  if (seqAbort) { landEmergency(); return; }

  holdAtHeight(TARGET_HEIGHT_M, 300);                  if (seqAbort) { landEmergency(); return; }

  flyBodyVX(-FWD_SPEED_MPS, TARGET_HEIGHT_M, SEGMENT_TIME_MS);
  if (seqAbort) { landEmergency(); return; }

  landToZero(RAMP_TIME_MS);
  DEBUG_PRINT("Sequence done\n");
}

void appMain(void) {
  DEBUG_PRINT("AUX0<%d triggers motion; abort if ranging.distance0 > %u mm\n",
              AUX_ACTIVE_THRESH, DIST0_ABORT_MM);

  // Resolve required log IDs
  while (!logVarIdIsValid(idAux0) || !logVarIdIsValid(idDistance0)) {
    ensureLogId(&idAux0,      "cppm",    "aux0");
    ensureLogId(&idDistance0, "ranging", "distance0");
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
