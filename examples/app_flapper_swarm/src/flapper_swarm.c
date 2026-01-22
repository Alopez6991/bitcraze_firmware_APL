/*
 * Unified Flapper Swarm App
 * 
 * This app supports multiple drones in a swarm with a single codebase.
 * The drone ID is derived from the radio address (last nibble of URI):
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
#include "param_logic.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"
#include "configblock.h"

// ============================================================================
// Drone ID (derived from radio address at startup)
// ============================================================================
static uint8_t droneId = 0;  // Will be set from radio address in appMain()

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
#define DES_DERIV -1400.0f          // desired derivative wrt middle beacon
#define RECOVER_YAWRATE 50.0f       // yawrate at 0 derivative
#define RECOVER_DEADZONE 30.0f      // stop rotating within this target yawrate

# 
// Drone 1 specific: RC trigger threshold (active low)
#define AUX_RC_ACTIVE_THRESH 1400
// Drone 2+ specific: UWB trigger threshold (active high)
#define AUX_UWB_ACTIVE_THRESHOLD 0U

// Height threshold below which we consider a drone "landed" (meters)
#define PEER_LANDED_HEIGHT_M 0.1f

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
static logVarId_t idHeight2     = (logVarId_t)0xFFFF;  // height of drone 2

// Drone 2+ specific (triggered via UWB)
static logVarId_t idRangingAux1 = (logVarId_t)0xFFFF;  // UWB trigger
static logVarId_t idRangingAux2 = (logVarId_t)0xFFFF;  // UWB kill switch
static logVarId_t idDistance1   = (logVarId_t)0xFFFF;  // distance to drone 1
static logVarId_t idHeight1     = (logVarId_t)0xFFFF;  // height of drone 1

// ============================================================================
// State machine
// ============================================================================
typedef enum {
  STATE_STRAIGHT = 0,
  STATE_TURN     = 1,
  STATE_AVOID    = 2,
  STATE_RECOVER  = 3
} FlightState;

static uint8_t currentState = STATE_STRAIGHT;  // Exposed for logging

// ============================================================================
// State variables
// ============================================================================
static uint8_t abortOverCount   = 0;
static uint8_t innerOverCount   = 0;
static uint8_t innerUnderCount  = 0;

// Avoidance confirmation counters
static uint8_t approachCount = 0;
static uint8_t departCount = 0;

// Sequence abort flag set by emergency check
static volatile bool seqAbort = false;

// Recover variables
static const float recoverFactor = - ( RECOVER_YAWRATE / DES_DERIV);
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

// Get the height of the peer drone (for avoidance)
static inline float getPeerHeight(void) {
  if (droneId == 1) {
    return logGetFloat(idHeight2);  // Drone 1 watches drone 2
  } else {
    return logGetFloat(idHeight1);  // Drone 2 watches drone 1
  }
}

// Check if the peer drone has landed (z < threshold)
static inline bool isPeerLanded(void) {
  float peerHeight = getPeerHeight();
  return (peerHeight >= 0.0f && peerHeight < PEER_LANDED_HEIGHT_M);
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

// Check if peer is too close and we should enter AVOID state
// Returns true if should enter AVOID (with confirmation)
static bool shouldEnterAvoid(void) {
  const uint32_t peerDist = getPeerDistance();
  
  // Don't enter avoid if peer has landed (z < threshold)
  if (isPeerLanded()) {
    approachCount = 0;  // Reset counter since we're not tracking
    return false;
  }
  
  if (peerDist > 0 && peerDist <= peerCloseMm) {
    if (++approachCount >= avoidEnterConfirmCount) {
      approachCount = 0;
      departCount = 0;
      return true;
    }
  } else {
    approachCount = 0;
  }
  return false;
}

// Check if peer is far enough and we should exit AVOID state
// Returns true if should exit AVOID (with confirmation)
static bool shouldExitAvoid(void) {
  const uint32_t peerDist = getPeerDistance();
  
  // Exit avoid immediately if peer has landed (no collision risk)
  if (isPeerLanded()) {
    departCount = 0;
    approachCount = 0;
    return true;
  }
  
  if (peerDist >= peerCloseMm) {
    if (++departCount >= avoidExitConfirmCount) {
      departCount = 0;
      approachCount = 0;
      return true;
    }
  } else {
    departCount = 0;
  }
  return false;
}

// Check for emergency land condition during AVOID
static bool checkAvoidEmergencyLand(void) {
  const uint32_t peerDist = getPeerDistance();
  
  if (currentState == STATE_AVOID && peerDist > 0 && peerDist <= avoidMinLandMm) {
    DEBUG_PRINT("AVOID EMERGENCY LAND: peer=%lu mm <= %u mm\n",
                (unsigned long)peerDist, avoidMinLandMm);
    seqAbort = true;
    return true;
  }
  return false;
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
// State machine context (shared between state handlers)
// ============================================================================
typedef struct {
  // Arc tracking (TURN state)
  bool arcActive;
  bool arcCooldown;
  float arcYawStart;
  float targetYaw;
  int rotationDirection;
  
  // Counters
  uint8_t routines;
  
  // Current sensor readings (updated each loop)
  uint32_t d0;
  float d0Deriv;
} StateContext;

static StateContext ctx;

// ============================================================================
// State handlers: onEnter, onExit, execute, checkTransition
// ============================================================================

// --- STRAIGHT state ---
static void onEnterStraight(void) {
  innerOverCount = 0;
  DEBUG_PRINT("Enter STRAIGHT\n");
}

static void onExitStraight(void) {
  // Nothing special to clean up
}

static FlightState checkTransitionStraight(void) {
  // STRAIGHT -> AVOID: peer too close
  if (shouldEnterAvoid()) {
    DEBUG_PRINT("STRAIGHT -> AVOID: peer too close\n");
    return STATE_AVOID;
  }
  
  // STRAIGHT -> TURN: reached outer bound (only if not in arc cooldown)
  if (ctx.d0 >= innerBoundMm && !ctx.arcCooldown) {
    if (++innerOverCount >= abortConfirmCount) {
      DEBUG_PRINT("STRAIGHT -> TURN (d0=%lu)\n", (unsigned long)ctx.d0);
      return STATE_TURN;
    }
  } else if (ctx.d0 < innerBoundMm) {
    innerOverCount = 0;
  }
  
  // STRAIGHT -> RECOVER: outside inner bound and moving away from beacon
  if (ctx.d0 >= innerBoundMm && ctx.d0Deriv > 0 && ctx.arcCooldown) {
    DEBUG_PRINT("STRAIGHT -> RECOVER: outside bound and deriv=%.2f (moving away)\n", (double)ctx.d0Deriv);
    return STATE_RECOVER;
  }
  
  return STATE_STRAIGHT;  // No transition
}

static void executeStraight(void) {
  sendHover(fwdSpeedMps, 0.0f, targetHeightM, 0.0f);
}

// --- TURN state ---
static void onEnterTurn(void) {
  innerUnderCount = 0;
  ctx.arcYawStart = logGetFloat(idYaw);
  ctx.targetYaw = normalizeAngle(ctx.arcYawStart - 90.0f);
  ctx.arcActive = isfinite(ctx.arcYawStart);
  DEBUG_PRINT("Enter TURN: arcActive=%d, startYaw=%.3f deg, targetYaw=%.3f deg\n",
              ctx.arcActive ? 1 : 0, (double)ctx.arcYawStart, (double)ctx.targetYaw);
}

static void onExitTurn(void) {
  ctx.arcActive = false;
}

static FlightState checkTransitionTurn(void) {
  // TURN -> AVOID: peer too close (one way, cannot return to TURN)
  if (shouldEnterAvoid()) {
    DEBUG_PRINT("TURN -> AVOID: peer too close\n");
    return STATE_AVOID;
  }
  
  // TURN -> STRAIGHT: back inside inner bound
  if (ctx.d0 <= innerBoundMm) {
    if (++innerUnderCount >= abortConfirmCount) {
      ctx.routines++;
      DEBUG_PRINT("TURN -> STRAIGHT: routine %u complete (d0=%lu)\n", ctx.routines, (unsigned long)ctx.d0);
      return STATE_STRAIGHT;
    }
  } else {
    innerUnderCount = 0;
  }
  
  // Arc tracking: when 270° reached, decide STRAIGHT or RECOVER
  if (ctx.arcActive) {
    float curYaw = logGetFloat(idYaw);
    if (isfinite(curYaw)) {
      const bool reached = (fabsf(curYaw - ctx.targetYaw) <= 3.0f) ||
                           (fabsf(curYaw - ctx.targetYaw - 360.0f) <= 3.0f);
      if (reached) {
        ctx.arcActive = false;
        ctx.arcCooldown = true;
        if (ctx.d0Deriv <= 0) {
          DEBUG_PRINT("TURN -> STRAIGHT: arc complete, deriv=%.2f (approaching)\n", (double)ctx.d0Deriv);
          return STATE_STRAIGHT;
        } else {
          DEBUG_PRINT("TURN -> RECOVER: arc complete, deriv=%.2f (moving away)\n", (double)ctx.d0Deriv);
          return STATE_RECOVER;
        }
      }
    } else {
      ctx.arcActive = false;
      DEBUG_PRINT("TURN arc: yaw unavailable, stopping arc tracking\n");
    }
  }
  
  return STATE_TURN;  // No transition
}

static void executeTurn(void) {
  sendHover(fwdSpeedMps, 0.0f, targetHeightM, turnYawRateDps);
}

// --- AVOID state ---
static void onEnterAvoid(void) {
  DEBUG_PRINT("Enter AVOID\n");
}

static void onExitAvoid(void) {
  approachCount = 0;
  departCount = 0;
}

static FlightState checkTransitionAvoid(void) {
  // AVOID -> STRAIGHT or RECOVER: peer far enough
  if (shouldExitAvoid()) {
    if (ctx.d0 < innerBoundMm) {
      DEBUG_PRINT("AVOID -> STRAIGHT: peer far, inside bound (d0=%lu)\n", (unsigned long)ctx.d0);
      return STATE_STRAIGHT;
    } else {
      DEBUG_PRINT("AVOID -> RECOVER: peer far, outside bound (d0=%lu)\n", (unsigned long)ctx.d0);
      return STATE_RECOVER;
    }
  }
  
  return STATE_AVOID;  // No transition
}

static void executeAvoid(void) {
  sendHover(fwdSpeedMps * avoidSpeedFactor, 0.0f, targetHeightM, getAvoidYawRate());
}

// --- RECOVER state ---
static void onEnterRecover(void) {
  if (droneId != 1) {
    if (ctx.d0 > dist0AbortMm - 800) { // subject to tuning
      DEBUG_PRINT("Keeping direction after avoid\n");
      ctx.rotationDirection = -1;
    }
  }
  DEBUG_PRINT("Enter RECOVER\n");
}

static void onExitRecover(void) {
  ctx.rotationDirection = 1;
}

static FlightState checkTransitionRecover(void) {
  // RECOVER -> AVOID: peer too close
  if (shouldEnterAvoid()) {
    DEBUG_PRINT("RECOVER -> AVOID: peer too close\n");
    return STATE_AVOID;
  }
  
  // RECOVER -> STRAIGHT: back inside inner bound
  if (ctx.d0 > 0 && ctx.d0 <= innerBoundMm) {
    DEBUG_PRINT("RECOVER -> STRAIGHT: made it back to the circle (d0=%lu)\n", (unsigned long)ctx.d0);
    return STATE_STRAIGHT;
  }
  
  return STATE_RECOVER;  // No transition
}

static void executeRecover(void) {
  float yawCommand = recoverFactor * fabsf(ctx.d0Deriv - DES_DERIV);
  yawCommand = yawCommand > RECOVER_DEADZONE ? yawCommand : 0.0f;
  yawCommand *= ctx.rotationDirection;
  sendHover(fwdSpeedMps, 0.0f, targetHeightM, yawCommand);
}

// ============================================================================
// State machine dispatcher functions
// ============================================================================
static void onEnterState(FlightState state) {
  switch (state) {
    case STATE_STRAIGHT: onEnterStraight(); break;
    case STATE_TURN:     onEnterTurn();     break;
    case STATE_AVOID:    onEnterAvoid();    break;
    case STATE_RECOVER:  onEnterRecover();  break;
  }
}

static void onExitState(FlightState state) {
  switch (state) {
    case STATE_STRAIGHT: onExitStraight(); break;
    case STATE_TURN:     onExitTurn();     break;
    case STATE_AVOID:    onExitAvoid();    break;
    case STATE_RECOVER:  onExitRecover();  break;
  }
}

static FlightState checkTransition(FlightState state) {
  switch (state) {
    case STATE_STRAIGHT: return checkTransitionStraight();
    case STATE_TURN:     return checkTransitionTurn();
    case STATE_AVOID:    return checkTransitionAvoid();
    case STATE_RECOVER:  return checkTransitionRecover();
  }
  return state;
}

static void executeState(FlightState state) {
  switch (state) {
    case STATE_STRAIGHT: executeStraight(); break;
    case STATE_TURN:     executeTurn();     break;
    case STATE_AVOID:    executeAvoid();    break;
    case STATE_RECOVER:  executeRecover();  break;
  }
}

// ============================================================================
// Main flight sequence
// ============================================================================
static void runSequence(void) {
  // Reset all state
  seqAbort = false;
  innerOverCount = innerUnderCount = 0;
  approachCount = departCount = 0;
  
  // Reset state context
  memset(&ctx, 0, sizeof(ctx));
  ctx.rotationDirection = 1;
  
  // Reset derivative buffer
  d0BufferReset();
  
  // Initialize state machine
  currentState = STATE_STRAIGHT;
  onEnterState(currentState);

  uint32_t startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

  const char* yawDir = (droneId == 1) ? "CW" : "CCW";
  DEBUG_PRINT("Drone %u: fwd, TURN if d0>=%u; AVOID peer<=%u (%s yaw); land if d0>=%u\n",
              droneId, innerBoundMm, peerCloseMm, yawDir, dist0AbortMm);

  // Kill check before takeoff (drone 2+ only)
  if (checkKillAndDisarm()) return;

  // Takeoff
  rampToHeight(targetHeightM, RAMP_TIME_MS);

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

    // Check for AVOID emergency (peer too close during AVOID)
    if (checkAvoidEmergencyLand()) break;

    // Update context with current sensor readings
    ctx.d0 = logGetUint(idDistance0);
    d0BufferAdd(ctx.d0, now);
    ctx.d0Deriv = d0BufferGetDerivative();

    // Clear arc cooldown once we re-enter the inner circle
    if (ctx.arcCooldown && ctx.d0 > 0 && ctx.d0 <= innerBoundMm) {
      ctx.arcCooldown = false;
      DEBUG_PRINT("ARC cooldown cleared by inner re-entry (d0=%lu)\n", (unsigned long)ctx.d0);
    }

    // ========================================================================
    // State machine: check transitions and execute current state
    // ========================================================================
    FlightState nextState = checkTransition(currentState);
    
    if (nextState != currentState) {
      onExitState(currentState);
      currentState = nextState;
      onEnterState(currentState);
    }

    if (seqAbort) break;

    executeState(currentState);

    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }

  landToZero();
  if (seqAbort) {
    DEBUG_PRINT("Emergency landing\n");
  } else {
    DEBUG_PRINT("Finished demo in approx %u routines\n", ctx.routines);
  }
}

// ============================================================================
// Velocity Controller Gains Configuration
// ============================================================================
static void setVelocityControllerGains(void) {
  // We set the gains to ensure they are consistent, even if a flapper has different gains stored in memory.
  // X velocity gains
  paramVarId_t vxKpId = paramGetVarId("velCtlPid", "vxKp");
  paramVarId_t vxKiId = paramGetVarId("velCtlPid", "vxKi");
  paramVarId_t vxKdId = paramGetVarId("velCtlPid", "vxKd");
  paramVarId_t vxKFFId = paramGetVarId("velCtlPid", "vxKFF");

  // Y velocity gains
  paramVarId_t vyKpId = paramGetVarId("velCtlPid", "vyKp");
  paramVarId_t vyKiId = paramGetVarId("velCtlPid", "vyKi");
  paramVarId_t vyKdId = paramGetVarId("velCtlPid", "vyKd");
  paramVarId_t vyKFFId = paramGetVarId("velCtlPid", "vyKFF");

  // Z velocity gains
  paramVarId_t vzKpId = paramGetVarId("velCtlPid", "vzKp");
  paramVarId_t vzKiId = paramGetVarId("velCtlPid", "vzKi");
  paramVarId_t vzKdId = paramGetVarId("velCtlPid", "vzKd");
  paramVarId_t vzKFFId = paramGetVarId("velCtlPid", "vzKFF");

  // Set X velocity gains
  if (PARAM_VARID_IS_VALID(vxKFFId)) paramSetFloat(vxKFFId, 30.0f);
  if (PARAM_VARID_IS_VALID(vxKdId)) paramSetFloat(vxKdId, 0.0f);
  if (PARAM_VARID_IS_VALID(vxKiId)) paramSetFloat(vxKiId, 5.0f);
  if (PARAM_VARID_IS_VALID(vxKpId)) paramSetFloat(vxKpId, 20.0f);

  // Set Y velocity gains
  if (PARAM_VARID_IS_VALID(vyKFFId)) paramSetFloat(vyKFFId, 8.0f);
  if (PARAM_VARID_IS_VALID(vyKdId)) paramSetFloat(vyKdId, 0.0f);
  if (PARAM_VARID_IS_VALID(vyKiId)) paramSetFloat(vyKiId, 5.0f);
  if (PARAM_VARID_IS_VALID(vyKpId)) paramSetFloat(vyKpId, 12.0f);

  // Set Z velocity gains
  if (PARAM_VARID_IS_VALID(vzKFFId)) paramSetFloat(vzKFFId, 0.0f);
  if (PARAM_VARID_IS_VALID(vzKdId)) paramSetFloat(vzKdId, 0.0f);
  if (PARAM_VARID_IS_VALID(vzKiId)) paramSetFloat(vzKiId, 0.5f);
  if (PARAM_VARID_IS_VALID(vzKpId)) paramSetFloat(vzKpId, 20.0f);

  DEBUG_PRINT("Velocity controller gains set\n");
}

// ============================================================================
// App entry point
// ============================================================================
void appMain(void) {
  // Get drone ID from radio address (last nibble, like lpsTwrTag does)
  droneId = (uint8_t)(configblockGetRadioAddress() & 0xF);
  DEBUG_PRINT("Flapper Swarm App started, droneId=%u (from radio address)\n", droneId);

  
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
    // Drone 1: RC trigger and distance/height to drone 2
    while (!logVarIdIsValid(idCppmAux0) || !logVarIdIsValid(idDistance2) || !logVarIdIsValid(idHeight2)) {
      ensureLogId(&idCppmAux0,   "cppm", "aux0");
      ensureLogId(&idDistance2,  "ranging", "distance2");
      ensureLogId(&idHeight2,    "ranging", "height2");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    DEBUG_PRINT("Drone 1: RC trigger (cppm.aux0<%d), avoid on distance2<=%u (CW)\n",
      AUX_RC_ACTIVE_THRESH, peerCloseMm);
  } else {
    // Drone 2+: UWB trigger/kill and distance/height to drone 1
    while (!logVarIdIsValid(idRangingAux1) || !logVarIdIsValid(idRangingAux2) ||
           !logVarIdIsValid(idDistance1) || !logVarIdIsValid(idHeight1)) {
      ensureLogId(&idRangingAux1, "ranging", "aux1");
      ensureLogId(&idRangingAux2, "ranging", "aux2");
      ensureLogId(&idDistance1,   "ranging", "distance1");
      ensureLogId(&idHeight1,     "ranging", "height1");
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    DEBUG_PRINT("Drone %u: UWB trigger (ranging.aux1>%u), kill (ranging.aux2), avoid on distance1<=%u (CCW)\n",
      droneId, AUX_UWB_ACTIVE_THRESHOLD, peerCloseMm);
  }
      
  bool wasActive = false;

  if (droneId != 0) {
    while (1) {
      const bool active = isTriggerActive();
  
      // Emergency always active, even when idle
      if (checkAndMaybeEmergencyLand()) {
        landToZero();
      }
      
      // On rising edge of trigger, run the sequence
      if (active && !wasActive) {
        // Set velocity controller gains to ensure consistent behavior
        setVelocityControllerGains();
        runSequence();
      }
      
      wasActive = active;
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

// ============================================================================
// Parameter definitions (can be changed from client)
// ============================================================================
PARAM_GROUP_START(swarm)
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

// ============================================================================
// Log definitions (for monitoring state)
// State values: 0=STRAIGHT, 1=TURN, 2=AVOID, 3=RECOVER
// ============================================================================
LOG_GROUP_START(swarm)
  LOG_ADD(LOG_UINT8, state, &currentState)
LOG_GROUP_STOP(swarm)
