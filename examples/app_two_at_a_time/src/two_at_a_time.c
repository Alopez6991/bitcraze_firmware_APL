#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "TWOAT"
#include "debug.h"

#include "log.h"
#include "commander.h"
#include "stabilizer_types.h"

// ---------------- Configuration via compile-time defines ----------------
// Build one of:
//   -DDRONE_01               // local RC aux1 triggers takeoff/hover/land
//   -DDRONE_02               // watches peer UWB aux mask
//
// If DRONE_02:
//   -DPEER_INDEX_TO_WATCH=N  // 0..4, which peer auxMaskN to read
//   -DTRIGGER_AUX_BIT=B      // 0..3, which aux bit to trigger on (default 1 = aux1)

#ifndef TAKEOFF_THRUST
#define TAKEOFF_THRUST  41000  // adjust to your platform
#endif
#ifndef HOVER_THRUST
#define HOVER_THRUST    38000  // approximate hover
#endif
#ifndef MAX_THRUST
#define MAX_THRUST      65000
#endif

#ifndef PEER_INDEX_TO_WATCH
#define PEER_INDEX_TO_WATCH 0
#endif

#ifndef TRIGGER_AUX_BIT
#define TRIGGER_AUX_BIT 1  // aux1 by default
#endif

#ifndef LOOP_DT_MS
#define LOOP_DT_MS 10
#endif

#ifndef TAKEOFF_RAMP_MS
#define TAKEOFF_RAMP_MS 1000
#endif

#ifndef HOVER_MS
#define HOVER_MS 3000
#endif

#ifndef LAND_RAMP_MS
#define LAND_RAMP_MS 1200
#endif

// ---------------- Helpers to read trigger sources ----------------

static logVarId_t idCppmAux1 = LOG_VAR_ID_INVALID;
static inline bool readLocalAux1LowActive(void) {
  if (!logVarIdIsValid(idCppmAux1)) {
    idCppmAux1 = logGetVarId("cppm", "aux1");
    if (!logVarIdIsValid(idCppmAux1)) {
      return false;
    }
  }
  // Consider "active" when < 1400us like switch.c
  const uint16_t v = logGetUint(idCppmAux1);
  return (v > 0 && v < 1400);
}

static logVarId_t idPeerAuxMask = LOG_VAR_ID_INVALID;
static inline uint8_t readPeerAuxMask(void) {
#ifdef DRONE_02
  if (!logVarIdIsValid(idPeerAuxMask)) {
    char name[16];
    snprintf(name, sizeof(name), "auxMask%d", (int)PEER_INDEX_TO_WATCH);
    idPeerAuxMask = logGetVarId("ranging", name);
    if (!logVarIdIsValid(idPeerAuxMask)) {
      return 0;
    }
  }
  return (uint8_t)logGetUint(idPeerAuxMask);
#else
  return 0;
#endif
}

static inline bool triggerRequested(void) {
#ifdef DRONE_01
  return readLocalAux1LowActive();
#elif defined(DRONE_02)
  uint8_t mask = readPeerAuxMask();
  return (mask & (1u << TRIGGER_AUX_BIT)) != 0;
#else
  return false;
#endif
}

// ---------------- Simple thrust flight routine ----------------

typedef enum {
  ST_IDLE = 0,
  ST_TAKEOFF,
  ST_HOVER,
  ST_LAND,
  ST_DONE
} flight_state_t;

static void sendThrustSetpoint(uint16_t thrust) {
  if (thrust > MAX_THRUST) thrust = MAX_THRUST;

  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));
  // Stabilized attitude = 0, thrust set
  sp.attitude.roll = 0.0f;
  sp.attitude.pitch = 0.0f;
  sp.attitude.yaw = 0.0f;
  sp.thrust = thrust;

  // Send setpoint continuously in the loop
  commanderSetSetpoint(&sp, xTaskGetTickCount());
}

static void flightRoutineOnce(void) {
  // Ramps and holds over time, calling sendThrustSetpoint() every LOOP_DT_MS
  const int takeoffSteps = TAKEOFF_RAMP_MS / LOOP_DT_MS;
  const int hoverSteps   = HOVER_MS / LOOP_DT_MS;
  const int landSteps    = LAND_RAMP_MS / LOOP_DT_MS;

  // Takeoff ramp
  for (int i = 0; i < takeoffSteps; i++) {
    uint16_t thr = (uint16_t)(HOVER_THRUST + (TAKEOFF_THRUST - HOVER_THRUST) * (float)i / (float)takeoffSteps);
    sendThrustSetpoint(thr);
    vTaskDelay(M2T(LOOP_DT_MS));
  }

  // Hover hold
  for (int i = 0; i < hoverSteps; i++) {
    sendThrustSetpoint(HOVER_THRUST);
    vTaskDelay(M2T(LOOP_DT_MS));
  }

  // Land ramp
  for (int i = 0; i < landSteps; i++) {
    uint16_t thr = (uint16_t)(HOVER_THRUST * (1.0f - (float)i / (float)landSteps));
    sendThrustSetpoint(thr);
    vTaskDelay(M2T(LOOP_DT_MS));
  }

  // Motors off
  sendThrustSetpoint(0);
}

// ---------------- App entry ----------------

void appMain(void) {
#if defined(DRONE_01)
  DEBUG_PRINT("two_at_a_time: DRONE_01 (RC aux1 triggers)\n");
#elif defined(DRONE_02)
  DEBUG_PRINT("two_at_a_time: DRONE_02 (watching ranging.auxMask%d bit %d)\n", (int)PEER_INDEX_TO_WATCH, (int)TRIGGER_AUX_BIT);
#else
  DEBUG_PRINT("two_at_a_time: define DRONE_01 or DRONE_02 at build time!\n");
#endif

  flight_state_t st = ST_IDLE;
  bool lastTrig = false;

  while (1) {
    bool trig = triggerRequested();
    bool rising = (trig && !lastTrig);
    lastTrig = trig;

    switch (st) {
      case ST_IDLE:
        if (rising) {
          DEBUG_PRINT("Trigger received: takeoff sequence\n");
          st = ST_TAKEOFF;
        }
        break;

      case ST_TAKEOFF:
        flightRoutineOnce();
        st = ST_DONE;
        DEBUG_PRINT("Sequence done\n");
        break;

      case ST_HOVER:
      case ST_LAND:
      case ST_DONE:
      default:
        // Stay done; re-arm on next rising edge if you prefer:
        if (rising) {
          DEBUG_PRINT("Re-triggered, starting again\n");
          st = ST_TAKEOFF;
        }
        break;
    }

    vTaskDelay(M2T(LOOP_DT_MS));
  }
}