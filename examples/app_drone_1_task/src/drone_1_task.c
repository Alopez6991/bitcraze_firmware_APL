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

static logVarId_t idAux0 = (logVarId_t)0xFFFF;

static inline bool aux0ActiveLow(void) {
  if (!logVarIdIsValid(idAux0)) {
    idAux0 = logGetVarId("cppm", "aux0");
    if (!logVarIdIsValid(idAux0)) {
      return false;
    }
  }
  const int16_t v = logGetInt(idAux0);
  return (v > 0) && (v < AUX_ACTIVE_THRESH);
}

static void sendHover(float vx, float vy, float z, float yawRateDeg) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));

  // Hold altitude at absolute z
  sp.mode.z = modeAbs;
  sp.position.z = z;

  // Body-frame XY velocity
  sp.mode.x = modeVelocity;
  sp.mode.y = modeVelocity;
  sp.velocity_body = true;
  sp.velocity.x = vx;
  sp.velocity.y = vy;

  // Yaw rate (deg/s)
  sp.mode.yaw = modeVelocity;
  sp.attitudeRate.yaw = yawRateDeg;

  commanderSetSetpoint(&sp, 3);
}

static void rampToHeight(float zTarget, uint32_t rampMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = (rampMs / dtMs) ? (rampMs / dtMs) : 1;
  for (uint32_t i = 0; i <= steps; i++) {
    const float z = (zTarget * (float)i) / (float)steps;
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

static void holdAtHeight(float z, uint32_t holdMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = holdMs / dtMs;
  for (uint32_t i = 0; i < steps; i++) {
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

static void flyBodyVX(float vx, float z, uint32_t durationMs) {
  const uint32_t dtMs = 20;
  const uint32_t steps = durationMs / dtMs;
  for (uint32_t i = 0; i < steps; i++) {
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
  // Send a few zero setpoints to fully stop
  for (int i = 0; i < 20; i++) {
    sendHover(0.0f, 0.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void runSequence(void) {
  DEBUG_PRINT("Sequence start: takeoff -> fwd -> stop -> back -> land\n");

  // Takeoff
  rampToHeight(TARGET_HEIGHT_M, RAMP_TIME_MS);
  holdAtHeight(TARGET_HEIGHT_M, 300);

  // Forward in body frame
  flyBodyVX(FWD_SPEED_MPS, TARGET_HEIGHT_M, SEGMENT_TIME_MS);

  // Brief stop
  holdAtHeight(TARGET_HEIGHT_M, 300);

  // Backward in body frame
  flyBodyVX(-FWD_SPEED_MPS, TARGET_HEIGHT_M, SEGMENT_TIME_MS);

  // Land
  landToZero(RAMP_TIME_MS);

  DEBUG_PRINT("Sequence done\n");
}

void appMain(void) {
  DEBUG_PRINT("drone_1_task: AUX0<%d triggers takeoff/forward/back/land\n", AUX_ACTIVE_THRESH);

  // Resolve var id (retry until available)
  while (!logVarIdIsValid(idAux0)) {
    idAux0 = logGetVarId("cppm", "aux0");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Visual indicator: LED on when idle
  ledSet(LED_BLUE_L, true);

  bool wasActive = false;
  while (1) {
    const bool active = aux0ActiveLow();

    // On rising edge of trigger, run the sequence
    if (active && !wasActive) {
      ledSet(LED_BLUE_L, false);  // turn off while running
      runSequence();
      ledSet(LED_BLUE_L, true);   // back on when done
    }

    wasActive = active;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
