/*
 * Unified Flapper Swarm App
 * 
 * This app supports multiple drones in a swarm with a single codebase.
 * The behavior is selected at runtime via the persistent parameter `swarm.droneId`:
 *   - droneId = 1: Primary drone, triggered by RC remote (cppm.aux0), avoids using distance2, CW yaw
 *   - droneId = 2: Secondary drone, triggered via UWB (ranging.aux1), avoids using distance1, CCW yaw
 * 
 * Common behavior:
 *   - All drones monitor distance0 (beacon) for outer boundary emergency land
 *   - All drones perform the same flight pattern (STRAIGHT -> TURN -> RECOVER)
 *   - Kill switch via ranging.aux2 (for droneId >= 2)
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "FLAPPERSWARM"
#include "debug.h"

#include "log.h"
#include "param.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"

// ============================================================================
// Runtime-configurable drone ID (persistent parameter, set from client)
// ============================================================================
static uint8_t droneId = 1;  // Default to drone 1

// ============================================================================
// Runtime-configurable flight parameters (can be changed from client)
// ============================================================================
static float targetHeightM = 1.0f;
static float fwdSpeedMps = 0.5f;
static uint16_t dist0AbortMm = 4200U;        // outer emergency bound to beacon (mm)
static uint16_t innerBoundMm = 1750U;        // Inner bound to start turning
static float turnYawRateDps = 40.0f;         // Yaw rate while turning (deg/s)
static uint16_t peerCloseMm = 2000U;         // near-limit on peer distance (2m)
static uint8_t abortConfirmCount = 2;        // Require N consecutive samples to trigger thresholds
static uint8_t avoidEnterConfirmCount = 2;
static uint8_t avoidExitConfirmCount = 4;
static uint16_t avoidMinLandMm = 600U;       // Drone min land distance
static float avoidSpeedFactor = 1.0f;        // full speed during avoidance
static float avoidYawRateMagnitude = 70.0f;  // absolute yaw rate for avoidance (deg/s)
static uint32_t demoTimeMs = 60000U;         // time of the demo in ms

// ============================================================================
// Fixed parameters (not runtime configurable)
// ============================================================================
#define RAMP_TIME_MS 1500U
#define LAND_VZ_MPS 0.4f            // descent speed
#define CUT_Z_M 0.05f               // cut controllers below this altitude
#define DERIV_SAMPLE_INTERVAL_MS 20
#define D0_BUFFER_SIZE 10

// Drone 1 specific: RC trigger threshold (active low)
#define AUX_RC_ACTIVE_THRESH 1400
// Drone 2+ specific: UWB trigger threshold (active high)
#define AUX_UWB_ACTIVE_THRESHOLD 0U

// ============================================================================
// Log variable IDs
// ============================================================================
// Common
static logVarId_t idDistance0   = (logVarId_t)0xFFFF;  // distance to beacon
static logVarId_t idZ           = (logVarId_t)0xFFFF;
static logVarId_t idYaw         = (logVarId_t)0xFFFF;

// Drone 1 specific
static logVarId_t idCppmAux0    = (logVarId_t)0xFFFF;  // RC trigger
static logVarId_t idDistance2   = (logVarId_t)0xFFFF;  // distance to drone 2

// Drone 2+ specific (triggered via UWB)
static logVarId_t idRangingAux1 = (logVarId_t)0xFFFF;  // UWB trigger
static logVarId_t idRangingAux2 = (logVarId_t)0xFFFF;  // UWB kill switch
static logVarId_t idDistance1   = (logVarId_t)0xFFFF;  // distance to drone 1

// ============================================================================
// State variables
// ============================================================================
static uint8_t abortOverCount   = 0;
static uint8_t innerOverCount   = 0;
static uint8_t innerUnderCount  = 0;

// Avoidance state
static bool avoidActive = false;
static uint8_t approachCount = 0;
static uint8_t departCount = 0;
static bool avoidWasActive = false;

// Sequence abort flag set by emergency check
static volatile bool seqAbort = false;

// ============================================================================
// Derivative buffer for d0
// ============================================================================
typedef struct {
  uint32_t samples[D0_BUFFER_SIZE];
  uint8_t writeIndex;
  uint8_t count;
  uint32_t lastSampleTime;
} D0Buffer;

static D0Buffer d0Buffer;

static void d0BufferReset(void) {
  memset(&d0Buffer, 0, sizeof(d0Buffer));
}

static void d0BufferAdd(uint32_t d0, uint32_t now) {
  if ((now - d0Buffer.lastSampleTime) >= DERIV_SAMPLE_INTERVAL_MS) {
    d0Buffer.samples[d0Buffer.writeIndex] = d0;
    d0Buffer.writeIndex = (d0Buffer.writeIndex + 1) % D0_BUFFER_SIZE;
    if (d0Buffer.count < D0_BUFFER_SIZE) {
      d0Buffer.count++;
    }
    d0Buffer.lastSampleTime = now;
  }
}

// Compute derivative using linear regression (least squares fit)
// Returns derivative in mm/s, positive = moving away, negative = moving toward
static float d0BufferGetDerivative(void) {
  if (d0Buffer.count < 2) {
    return 0.0f;
  }
  
  uint8_t n = d0Buffer.count;
  
  float sumT = 0.0f;
  float sumY = 0.0f;
  float sumTY = 0.0f;
  float sumT2 = 0.0f;
  
  for (uint8_t i = 0; i < n; i++) {
    uint8_t idx;
    if (d0Buffer.count < D0_BUFFER_SIZE) {
      idx = i;
    } else {
      idx = (d0Buffer.writeIndex + i) % D0_BUFFER_SIZE;
    }
    
    float t = (float)i;
    float y = (float)d0Buffer.samples[idx];
    
    sumT += t;
    sumY += y;
    sumTY += t * y;
    sumT2 += t * t;
  }
  
  float denom = (float)n * sumT2 - sumT * sumT;
  if (fabsf(denom) < 1e-6f) {
    return 0.0f;
  }
  
  float slopePerInterval = ((float)n * sumTY - sumT * sumY) / denom;
  float intervalS = (float)DERIV_SAMPLE_INTERVAL_MS / 1000.0f;
  
  return slopePerInterval / intervalS;
}

// ============================================================================
// Utility functions
// ============================================================================
static inline void ensureLogId(logVarId_t* id, const char* group, const char* name) {
  if (!logVarIdIsValid(*id)) {
    *id = logGetVarId(group, name);
  }
}

static inline float normalizeAngle(float a) {
  while (a <= -180) a += 360;
  while (a > 180) a -= 360;
  return a;
}

// ============================================================================
// Drone-specific behavior (runtime selection based on droneId)
// ============================================================================

// Check if the trigger is active (start signal)
static inline bool isTriggerActive(void) {
  if (droneId == 1) {
    // Drone 1: RC trigger via cppm.aux0 (active low)
    const int16_t v = logGetInt(idCppmAux0);
    return (v > 0) && (v < AUX_RC_ACTIVE_THRESH);
  } else {
    // Drone 2+: UWB trigger via ranging.aux1 (active high)
    const uint32_t v = logGetUint(idRangingAux1);
    return v > AUX_UWB_ACTIVE_THRESHOLD;
  }
}

// Check if the kill switch is active (drone 2+ only)
static inline bool isKillActive(void) {
  if (droneId == 1) {
    // Drone 1 has no kill switch
    return false;
  } else {
    // Drone 2+: kill via ranging.aux2 (active high)
    const uint32_t v = logGetUint(idRangingAux2);
    return v > AUX_UWB_ACTIVE_THRESHOLD;
  }
}

// Get the distance to the peer drone (for avoidance)
static inline uint32_t getPeerDistance(void) {
  if (droneId == 1) {
    return logGetUint(idDistance2);  // Drone 1 watches drone 2
  } else {
    return logGetUint(idDistance1);  // Drone 2 watches drone 1
  }
}

// Get the avoidance yaw rate (sign depends on drone)
static inline float getAvoidYawRate(void) {
  if (droneId == 1) {
    return avoidYawRateMagnitude;   // CW (positive)
  } else {
    return -avoidYawRateMagnitude;  // CCW (negative)
  }
}

// Check kill switch and disarm if active
static inline bool checkKillAndDisarm(void) {
  if (isKillActive()) {
    supervisorRequestArming(false);
    commanderRelaxPriority();
    return true;
  }
  return false;
}

// ============================================================================
// Command helpers
// ============================================================================
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

// ============================================================================
// Emergency and avoidance logic
// ============================================================================
static bool checkAndMaybeEmergencyLand(void) {
  bool trigger = false;

  const uint32_t d0 = logGetUint(idDistance0);
  if (d0 > dist0AbortMm) {
    if (abortOverCount < 0xFF) abortOverCount++;
    if (abortOverCount >= abortConfirmCount) {
      if (!seqAbort) {
        DEBUG_PRINT("Emergency FAR: distance0=%lu mm (> %u)\n",
                    (unsigned long)d0, dist0AbortMm);
      }
      seqAbort = true;
      trigger = true;
    }
  } else {
    abortOverCount = 0;
  }

  return trigger;
}

static bool updateAvoidanceMode(void) {
  const uint32_t peerDist = getPeerDistance();

  // If already in avoidance and we get dangerously close, land immediately
  if (avoidActive && peerDist > 0 && peerDist <= avoidMinLandMm) {
    DEBUG_PRINT("AVOID EMERGENCY LAND: peer=%lu mm <= %u mm\n",
                (unsigned long)peerDist, avoidMinLandMm);
    seqAbort = true;
    return true;
  }

  if (!avoidActive) {
    if (peerDist > 0 && peerDist <= peerCloseMm) {
      if (++approachCount >= avoidEnterConfirmCount) {
        avoidActive = true;
        approachCount = 0;
        departCount = 0;
        DEBUG_PRINT("AVOID start: peer=%lu mm\n", (unsigned long)peerDist);
      }
    } else {
      approachCount = 0;
    }
  } else {
    if (peerDist >= peerCloseMm) {
      if (++departCount >= avoidExitConfirmCount) {
        avoidActive = false;
        departCount = 0;
        approachCount = 0;
        DEBUG_PRINT("AVOID stop: peer=%lu mm\n", (unsigned long)peerDist);
      }
    } else {
      departCount = 0;
    }
  }

  return avoidActive;
}

// ============================================================================
// Landing and takeoff
// ============================================================================
static void landToZero(void) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));
  sp.mode.x = modeVelocity; sp.velocity.x = 0;
  sp.mode.y = modeVelocity; sp.velocity.y = 0;
  sp.mode.z = modeVelocity; sp.velocity.z = -LAND_VZ_MPS;
  sp.mode.yaw = modeVelocity; sp.attitudeRate.yaw = 0;
  sp.velocity_body = true;

  while (1) {
    float z = logGetFloat(idZ);
    if (z >= 0.0f && z <= CUT_Z_M) break;
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  // Cut controllers and thrust
  setpoint_t cut;
  memset(&cut, 0, sizeof(cut));
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
    if (checkKillAndDisarm()) return;
    if (checkAndMaybeEmergencyLand()) return;
    const float z = (zTarget * (float)i) / (float)steps;
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

// ============================================================================
// Main flight sequence
// ============================================================================
static void runSequence(void) {
  seqAbort = false;
  innerOverCount = innerUnderCount = 0;
  uint8_t routines = 0;

  bool arcActive = false;
  bool arcCooldown = false;
  float arcYawStart = 0.0f;
  float target_yaw = 0.0f;

  d0BufferReset();

  uint32_t startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

  const char* yawDir = (droneId == 1) ? "CW" : "CCW";
  DEBUG_PRINT("Drone %u: fwd, TURN if d0>=%u; AVOID peer<=%u (%s yaw); land if d0>=%u\n",
              droneId, innerBoundMm, peerCloseMm, yawDir, dist0AbortMm);

  // Kill check before takeoff (drone 2+ only)
  if (checkKillAndDisarm()) return;

  // Takeoff
  rampToHeight(targetHeightM, RAMP_TIME_MS);

  enum { STRAIGHT = 0, TURN = 1, RECOVER = 2 } mode = STRAIGHT;
  const uint32_t dtMs = 20;

  while (!seqAbort) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - startTime) > demoTimeMs) {
      break;
    }

    // Kill check (drone 2+ only)
    if (checkKillAndDisarm()) break;

    // Emergency checks (outer bound)
    if (checkAndMaybeEmergencyLand()) break;

    // Read distance0
    uint32_t d0 = logGetUint(idDistance0);

    // Add to derivative buffer
    d0BufferAdd(d0, now);
    float d0Deriv = d0BufferGetDerivative();

    // Clear arc cooldown once we re-enter the inner circle
    if (arcCooldown && d0 > 0 && d0 <= innerBoundMm) {
      arcCooldown = false;
      DEBUG_PRINT("ARC cooldown cleared by inner re-entry (d0=%lu)\n", (unsigned long)d0);
    }

    // Mode transitions with confirmation
    if (mode == STRAIGHT) {
      if (d0 >= innerBoundMm) {
        if (!arcCooldown && (++innerOverCount >= abortConfirmCount)) {
          mode = TURN;
          innerOverCount = 0;
          arcYawStart = logGetFloat(idYaw);
          target_yaw = normalizeAngle(arcYawStart - 90.0f);
          arcActive = isfinite(arcYawStart);
          DEBUG_PRINT("ENTER TURN (d0=%lu), arcActive=%d, startYaw=%.3f deg\n",
                      (unsigned long)d0, arcActive ? 1 : 0, (double)arcYawStart);
        }
      } else {
        innerOverCount = 0;
      }
    } else if (mode == TURN) {
      if (d0 <= innerBoundMm) {
        if (++innerUnderCount >= abortConfirmCount) {
          mode = STRAIGHT;
          innerUnderCount = 0;
          routines++;
          DEBUG_PRINT("EXIT TURN -> routine %u complete (d0=%lu)\n", routines, (unsigned long)d0);
        }
      } else {
        innerUnderCount = 0;
      }
    }

    // Arc tracking: when 270° reached, stop yaw
    if (mode == TURN && arcActive) {
      float curYaw = logGetFloat(idYaw);
      if (isfinite(curYaw)) {
        const bool reached = (fabsf(curYaw - target_yaw) <= 3.0f) ||
                             (fabsf(curYaw - target_yaw - 360.0f) <= 3.0f);
        if (reached) {
          DEBUG_PRINT("TURN arc reached 270° -> stop yaw\n");
          mode = STRAIGHT;
          arcActive = false;
          arcCooldown = true;
          innerUnderCount = 0;
          if (d0Deriv > 0) {
            DEBUG_PRINT("However, derivative was %.2f so going into recovery mode\n", (double)d0Deriv);
            mode = RECOVER;
          }
        }
      } else {
        arcActive = false;
        DEBUG_PRINT("TURN arc: yaw unavailable, stopping arc tracking\n");
      }
    }

    // Avoidance overrides the normal command
    avoidWasActive = avoidActive;
    const bool avoid = updateAvoidanceMode();

    if ((avoidWasActive && !avoid) && (d0 >= innerBoundMm)) {
      mode = RECOVER;
    }

    if (seqAbort) break;

    // Send commands based on mode
    if (avoid) {
      sendHover(fwdSpeedMps * avoidSpeedFactor, 0.0f, targetHeightM, getAvoidYawRate());
    } else if (mode == STRAIGHT) {
      sendHover(fwdSpeedMps, 0.0f, targetHeightM, 0.0f);
    } else if (mode == RECOVER) {
      float yawCommand = (50.0f / 1400.0f) * fabsf(d0Deriv + 1400);
      yawCommand = yawCommand > 30.0f ? yawCommand : 0.0f;
      sendHover(fwdSpeedMps, 0.0f, targetHeightM, yawCommand);
      DEBUG_PRINT("Recovering with yawrate %.2f deg/s for deriv %.2f\n", (double)yawCommand, (double)d0Deriv);
      if (d0 > 0 && d0 <= innerBoundMm) {
        mode = STRAIGHT;
        DEBUG_PRINT("Made it back to the circle!\n");
      }
    } else {
      // TURN mode
      sendHover(fwdSpeedMps, 0.0f, targetHeightM, turnYawRateDps);
    }

    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }

  landToZero();
  if (seqAbort) {
    DEBUG_PRINT("Emergency landing\n");
  } else {
    DEBUG_PRINT("Finished demo in approx %u routines\n", routines);
  }
}

// ============================================================================
// App entry point
// ============================================================================
void appMain(void) {
  DEBUG_PRINT("Flapper Swarm App started, droneId=%u\n", droneId);

  // Resolve log IDs based on droneId
  // Common IDs for all drones
  while (!logVarIdIsValid(idDistance0) || !logVarIdIsValid(idZ) || !logVarIdIsValid(idYaw)) {
    ensureLogId(&idDistance0, "ranging", "distance0");
    ensureLogId(&idZ,         "stateEstimate", "z");
    ensureLogId(&idYaw,       "stateEstimate", "yaw");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Drone-specific IDs
  if (droneId == 1) {
    // Drone 1: RC trigger and distance to drone 2
    while (!logVarIdIsValid(idCppmAux0) || !logVarIdIsValid(idDistance2)) {
      ensureLogId(&idCppmAux0,   "cppm", "aux0");
      ensureLogId(&idDistance2,  "ranging", "distance2");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    DEBUG_PRINT("Drone 1: RC trigger (cppm.aux0<%d), avoid on distance2<=%u (CW)\n",
                AUX_RC_ACTIVE_THRESH, peerCloseMm);
  } else {
    // Drone 2+: UWB trigger/kill and distance to drone 1
    while (!logVarIdIsValid(idRangingAux1) || !logVarIdIsValid(idRangingAux2) ||
           !logVarIdIsValid(idDistance1)) {
      ensureLogId(&idRangingAux1, "ranging", "aux1");
      ensureLogId(&idRangingAux2, "ranging", "aux2");
      ensureLogId(&idDistance1,   "ranging", "distance1");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    DEBUG_PRINT("Drone %u: UWB trigger (ranging.aux1>%u), kill (ranging.aux2), avoid on distance1<=%u (CCW)\n",
                droneId, AUX_UWB_ACTIVE_THRESHOLD, peerCloseMm);
  }

  bool wasActive = false;
  while (1) {
    const bool active = isTriggerActive();

    // Emergency always active, even when idle
    if (checkAndMaybeEmergencyLand()) {
      landToZero();
    }

    // On rising edge of trigger, run the sequence
    if (active && !wasActive) {
      runSequence();
    }

    wasActive = active;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ============================================================================
// Parameter definitions (can be changed from client)
// ============================================================================
PARAM_GROUP_START(swarm)
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, droneId, &droneId)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, targetHeight, &targetHeightM)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, fwdSpeed, &fwdSpeedMps)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, dist0Abort, &dist0AbortMm)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, innerBound, &innerBoundMm)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, turnYawRate, &turnYawRateDps)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, peerClose, &peerCloseMm)
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, abortConfirm, &abortConfirmCount)
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, avoidEnter, &avoidEnterConfirmCount)
  PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, avoidExit, &avoidExitConfirmCount)
  PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, avoidMinLand, &avoidMinLandMm)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, avoidSpeed, &avoidSpeedFactor)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, avoidYawRate, &avoidYawRateMagnitude)
  PARAM_ADD(PARAM_UINT32 | PARAM_PERSISTENT, demoTime, &demoTimeMs)
PARAM_GROUP_STOP(swarm)
