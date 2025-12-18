/*
// filepath: /home/austin/crazyflie-firmware/examples/app_drone_2_task/src/drone_2_task.c
*/
#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "DRONE2TASK"
#include "debug.h"

#include "log.h"
#include "led.h"

static logVarId_t idRangingAux0 = (logVarId_t)0xFFFF;

static inline bool sharedAux0Active(void) {
  if (!logVarIdIsValid(idRangingAux0)) {
    idRangingAux0 = logGetVarId("ranging", "aux0");
    if (!logVarIdIsValid(idRangingAux0)) {
      return false;
    }
  }
  const uint32_t v = logGetUint(idRangingAux0);
  return v > 0;
}

void appMain(void) {
  DEBUG_PRINT("drone_2_task: ranging.aux0>0 => left blue LED OFF\n");

  // Resolve var id (retry until available)
  while (!logVarIdIsValid(idRangingAux0)) {
    idRangingAux0 = logGetVarId("ranging", "aux0");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Default ON so change is visible when aux0 becomes active
  ledSet(LED_BLUE_L, true);

  while (1) {
    const bool active = sharedAux0Active();
    // Active => turn OFF left blue LED, else ON
    ledSet(LED_BLUE_L, active ? false : true);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
