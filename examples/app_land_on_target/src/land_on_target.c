/* Minimal app that listens for an appchannel "LAND" command and prints when received
 * Use the tools/trigger.py to send the trigger packet from the host
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app_channel.h"

#define DEBUG_MODULE "LAND"

void appMain()
{
  DEBUG_PRINT("Land-on-target app started\n");

  while (1) {
    /* Wait forever for an incoming appchannel packet (blocking) */
    char buf[32];
    size_t r = appchannelReceiveDataPacket(buf, sizeof(buf), APPCHANNEL_WAIT_FOREVER);
    if (r > 0) {
      buf[(r < sizeof(buf)) ? r : (sizeof(buf)-1)] = '\0';
      DEBUG_PRINT("Received appchannel packet: %s\n", buf);
      if (strcmp(buf, "LAND") == 0) {
        DEBUG_PRINT("Landing triggered!\n");
        /* TODO: add landing logic here (commander commands) */
      }
    }
  }
}
