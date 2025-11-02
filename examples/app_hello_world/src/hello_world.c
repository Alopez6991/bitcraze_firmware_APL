/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"

#define DEBUG_MODULE "HELLOWORLD"
#include "debug.h"


/* Minimal switch-triggered hello world
 * - Reads cppm.aux3 log variable
 * - When aux3 goes below 1400 (B-switch down) we consider it active
 * - Print immediately on activation and then every 2000 ms while held
 */

static logVarId_t idAux3 = 0xffffu;
static bool isActive = false;

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  TickType_t lastPrint = 0;

  while (1) {
    /* Poll at 50 Hz so we react quickly to switch changes */
    vTaskDelay(F2T(50));

    if (!logVarIdIsValid(idAux3)) {
      idAux3 = logGetVarId("cppm", "aux3");
      continue;
    }

    int16_t aux3 = logGetInt(idAux3);

    /* aux3: ~2000 = UP (inactive), ~1000 = DOWN (active). Treat <1400 as active */
    bool nowActive = (aux3 > 0) && (aux3 < 1400);

    if (nowActive && !isActive) {
      /* Just activated: print immediately */
      DEBUG_PRINT("Hello World! (activated)\n");
      lastPrint = xTaskGetTickCount();
      isActive = true;
      continue;
    }

    if (!nowActive && isActive) {
      /* Just deactivated */
      DEBUG_PRINT("Hello World stopped\n");
      isActive = false;
      continue;
    }

    /* While active, print every 2000 ms */
    if (isActive) {
      if ((xTaskGetTickCount() - lastPrint) >= M2T(2000)) {
        DEBUG_PRINT("Hello World!\n");
        lastPrint = xTaskGetTickCount();
      }
    }
  }
}
