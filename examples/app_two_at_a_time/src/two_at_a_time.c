#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "TWOAT"
#include "debug.h"

#include "log.h"
#include "param.h"
#include "supervisor.h"
#include "crtp_commander_high_level.h"

// Build-time selection: set DRONE_NUMBER to 1 or 2
#ifndef DRONE_NUMBER
#define DRONE_NUMBER 1
#endif

#if (DRONE_NUMBER == 1)
  #ifndef DRONE_01
  #define DRONE_01
  #endif
#elif (DRONE_NUMBER == 2)
  #ifndef DRONE_02
  #define DRONE_02
  #endif
#else
  #ifndef DRONE_01
  #define DRONE_01
  #endif
#endif

#ifndef TARGET_HEIGHT_M
#define TARGET_HEIGHT_M 0.80f
#endif
#ifndef HOVER_TIME_MS
#define HOVER_TIME_MS 1500U
#endif
#ifndef ARM_TIMEOUT_MS
#define ARM_TIMEOUT_MS 2000U
#endif

#ifndef PEER_INDEX_TO_WATCH
#define PEER_INDEX_TO_WATCH 0
#endif
#define AUX_ACTIVE_THRESH 1400
#define AUX2_BIT 2

// IDs (0xFFFF = invalid in this firmware)
static logVarId_t idAux1 = (logVarId_t)0xFFFF;
#ifdef DRONE_02
static logVarId_t idPeerMask = (logVarId_t)0xFFFF;
#endif

static inline bool auxLowActive(logVarId_t id) {
  if (!logVarIdIsValid(id)) return false;
  const int16_t v = logGetInt(id);
  return (v > 0) && (v < AUX_ACTIVE_THRESH);
}

static inline bool triggerRequested(void) {
#ifdef DRONE_01
  if (!logVarIdIsValid(idAux1)) {
    idAux1 = logGetVarId("cppm", "aux0");
  }
  return auxLowActive(idAux1);
#elif defined(DRONE_02)
  if (!logVarIdIsValid(idPeerMask)) {
    char name[16];
    snprintf(name, sizeof(name), "auxMask%d", (int)PEER_INDEX_TO_WATCH);
    idPeerMask = logGetVarId("ranging", name);
  }
  if (!logVarIdIsValid(idPeerMask)) return false;
  const uint16_t m = logGetUint(idPeerMask);
  return (m & (1u << AUX2_BIT)) != 0;
#else
  return false;
#endif
}

#ifndef PARAM_VARID_IS_VALID
#define PARAM_VARID_IS_VALID(id) ((id) != (paramVarId_t)0xFFFF)
#endif

static inline void forceKalmanAndReset(void) {
  const paramVarId_t idEst   = paramGetVarId("stabilizer", "estimator");
  const paramVarId_t idReset = paramGetVarId("kalman", "resetEstimation");
  if (PARAM_VARID_IS_VALID(idEst))   { paramSetInt(idEst, 2); } // 2 = Kalman
  if (PARAM_VARID_IS_VALID(idReset)) { paramSetInt(idReset, 1); vTaskDelay(pdMS_TO_TICKS(100)); paramSetInt(idReset, 0); }
}

static inline bool waitForArmed(uint32_t ms) {
  const TickType_t t0 = xTaskGetTickCount();
  const TickType_t to = pdMS_TO_TICKS(ms);
  while (!supervisorIsArmed()) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if ((xTaskGetTickCount() - t0) > to) return false;
  }
  return true;
}

static void doTakeoffHoverLand(void) {
  forceKalmanAndReset();
  supervisorRequestArming(true);
  (void)waitForArmed(ARM_TIMEOUT_MS);

  crtpCommanderHighLevelTakeoff(TARGET_HEIGHT_M, 1.0f);
  vTaskDelay(pdMS_TO_TICKS(HOVER_TIME_MS));

  crtpCommanderHighLevelLand(0.0f, 0.5f);
  vTaskDelay(pdMS_TO_TICKS(1000));
  supervisorRequestArming(false);
}

void appMain(void) {
#if defined(DRONE_01)
  DEBUG_PRINT("two_at_a_time: DRONE_01 (trigger=cppm.aux1<%d)\n", AUX_ACTIVE_THRESH);
#elif defined(DRONE_02)
  DEBUG_PRINT("two_at_a_time: DRONE_02 (trigger=ranging.auxMask%d bit %d)\n", (int)PEER_INDEX_TO_WATCH, AUX2_BIT);
#else
  DEBUG_PRINT("two_at_a_time: define DRONE_NUMBER=1 or 2\n");
#endif

  bool lastTrig = false;
  while (1) {
    bool trig = triggerRequested();
    bool rising = (trig && !lastTrig);
    lastTrig = trig;

    if (rising) {
      DEBUG_PRINT("Trigger -> takeoff/land\n");
      doTakeoffHoverLand();
      DEBUG_PRINT("Done\n");
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}