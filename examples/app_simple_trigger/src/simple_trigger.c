#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app_channel.h"

#define DEBUG_MODULE "SIMPLE"
static bool isActive = false;

void appMain()
{
  DEBUG_PRINT("Waiting for START command on appchannel...\n");

  TickType_t lastPrint = 0;

  while (1) {
    /* Wait briefly for appchannel input so we can also handle periodic prints */
    char buf[32];
    size_t r = appchannelReceiveDataPacket(buf, sizeof(buf), F2T(50));
    if (r > 0) {
      /* Ensure null-termination */
      buf[(r < sizeof(buf)) ? r : (sizeof(buf)-1)] = '\0';
      DEBUG_PRINT("Received command: %s\n", buf);

      if (strcmp(buf, "START") == 0) {
        if (!isActive) {
          DEBUG_PRINT("Hello World! (activated)\n");
          lastPrint = xTaskGetTickCount();
          isActive = true;
        }
      } else if (strcmp(buf, "STOP") == 0) {
        if (isActive) {
          DEBUG_PRINT("Hello World stopped\n");
          isActive = false;
        }
      } else if (strcmp(buf, "TRIGGER") == 0) {
        DEBUG_PRINT("Hello World!\n");
      }
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
