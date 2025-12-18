/*
// filepath: /home/austin/crazyflie-firmware/examples/app_drone_1_task/src/drone_1_task.c
*/
#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "DRONE1TASK"
#include "debug.h"

#include "log.h"
#include "led.h"

#ifndef AUX_ACTIVE_THRESH
#define AUX_ACTIVE_THRESH 1400
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

void appMain(void) {
  DEBUG_PRINT("drone_1_task: AUX0<%d => left blue LED OFF\n", AUX_ACTIVE_THRESH);

  // Resolve var id (retry until available)
  while (!logVarIdIsValid(idAux0)) {
    idAux0 = logGetVarId("cppm", "aux0");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Default ON so change is visible when AUX0 becomes active
  ledSet(LED_BLUE_L, true);

  while (1) {
    const bool active = aux0ActiveLow();
    // Active => turn OFF left blue LED, else ON
    ledSet(LED_BLUE_L, active ? false : true);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
