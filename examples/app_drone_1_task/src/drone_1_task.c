/*
// filepath: /home/austin/crazyflie-firmware/examples/app_drone_1_task/src/drone_1_task.c
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "DRONE1TASK"
#include "debug.h"

#include "log.h"
#include "commander.h"
#include "stabilizer_types.h"

#define AUX_ACTIVE_THRESH 1400
#define TARGET_HEIGHT_M 1.0f
#define FWD_SPEED_MPS 0.5f
#define SEGMENT_TIME_MS 8000U
#define RAMP_TIME_MS 1500U
#define DIST0_ABORT_MM 4200U   // outer emergency bound (mm)
#define DIST0_HYST_MM 100U          // hysteresis margin
#define INNER_BOUND_MM 1750U // Inner bound to start turning
#define INNER_HYST_MM 100U
#define TURN_YAW_RATE_DPS 40.0f // Yaw rate while turning (deg/s)
#define LAND_VZ_MPS 0.4f            // descent speed
#define CUT_Z_M 0.05f               // cut controllers below this altitude
#define DIST2_CLOSE_MM 2000U // near-limit on distance2 (2m)
#define DIST2_HYST_MM 100U
#define ABORT_CONFIRM_COUNT 2 // Require N consecutive samples to trigger thresholds
#define AVOID_ENTER_CONFIRM_COUNT 2
#define AVOID_EXIT_CONFIRM_COUNT 4
#define AVOID_MIN_LAND_MM 600U
#define AVOID_SPEED_FACTOR 1.0f      // full speed during avoidance
#define AVOID_YAW_RATE_DPS 70.0f     // CW yaw rate for avoidance
#define DIST2_AVOID_MM DIST2_CLOSE_MM
#define DIST2_AVOID_HYST_MM DIST2_HYST_MM
#define AVOID_CONFIRM_COUNT ABORT_CONFIRM_COUNT
#define DEMO_TIME_MS 60000U // time of the demo in ms
#define DERIV_SAMPLE_INTERVAL_MS 20
#define D0_BUFFER_SIZE 10

static logVarId_t idAux0 = (logVarId_t)0xFFFF;
static logVarId_t idDistance0 = (logVarId_t)0xFFFF; // distance to the beacon
static logVarId_t idZ = (logVarId_t)0xFFFF;
static logVarId_t idYaw = (logVarId_t)0xFFFF;
static logVarId_t idDistance2 = (logVarId_t)0xFFFF; // distance to the other drone

static uint8_t abortOverCount = 0;
// New: inner bound enter/exit confirmation
static uint8_t innerOverCount = 0;
static uint8_t innerUnderCount = 0;

// Avoidance state
static bool avoidActive = false;
static uint8_t approachCount = 0;
static uint8_t departCount = 0;
bool avoidWasActive = false;

// Derivative state
typedef struct {
  uint32_t samples[D0_BUFFER_SIZE];
  uint8_t writeIndex;
  uint8_t count;
  uint32_t lastSampleTime;
} D0Buffer;

static D0Buffer d0Buffer;

// Initialize the d0 buffer
static void d0BufferReset(void) {
  memset(&d0Buffer, 0, sizeof(d0Buffer));
}

// Add a sample to the buffer
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
// Returns 0 if not enough samples
static float d0BufferGetDerivative(void) {
  if (d0Buffer.count < 2) {
    // Need at least 2 samples
    return 0.0f;
  }
  
  // Use available samples (may be less than D0_BUFFER_SIZE during warmup)
  uint8_t n = d0Buffer.count;
  
  // Linear regression: fit y = a + b*t to the data
  // b = (n*sum(t*y) - sum(t)*sum(y)) / (n*sum(t^2) - (sum(t))^2)
  // where t is time index (0, 1, 2, ..., n-1) and y is distance
  
  float sumT = 0.0f;
  float sumY = 0.0f;
  float sumTY = 0.0f;
  float sumT2 = 0.0f;
  
  for (uint8_t i = 0; i < n; i++) {
    // Get sample index in circular buffer (oldest to newest)
    uint8_t idx;
    if (d0Buffer.count < D0_BUFFER_SIZE) {
      // Buffer not yet full, samples start at index 0
      idx = i;
    } else {
      // Buffer full, oldest is at writeIndex
      idx = (d0Buffer.writeIndex + i) % D0_BUFFER_SIZE;
    }
    
    float t = (float)i;  // time index (in units of sample intervals)
    float y = (float)d0Buffer.samples[idx];
    
    sumT += t;
    sumY += y;
    sumTY += t * y;
    sumT2 += t * t;
  }
  
  float denom = (float)n * sumT2 - sumT * sumT;
  if (fabsf(denom) < 1e-6f) {
    return 0.0f;  // Avoid division by zero
  }
  
  // Slope in mm per sample interval
  float slopePerInterval = ((float)n * sumTY - sumT * sumY) / denom;
  
  // Convert to mm/s
  float intervalS = (float)DERIV_SAMPLE_INTERVAL_MS / 1000.0f;
  float derivativeMmPerS = slopePerInterval / intervalS;
  
  return derivativeMmPerS;
}

// Sequence abort flag set by emergency check
static volatile bool seqAbort = false;

// Resolve a log var id if needed
static inline void ensureLogId(logVarId_t* id, const char* group, const char* name) {
  if (!logVarIdIsValid(*id)) {
    *id = logGetVarId(group, name);
  }
}

// Normalize angle to (-180, 180] deg
static inline float normalizeAngle(float a) {
  while (a <= -180) a += 360;
  while (a > 180) a -= 360;
  return a;
}

static inline bool aux0ActiveLow(void) {
  const int16_t v = logGetInt(idAux0);
  return (v > 0) && (v < AUX_ACTIVE_THRESH);
}

// Send a hover/vel setpoint (body frame XY, absolute Z)
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

static bool checkAndMaybeEmergencyLand(void) {
  bool trigger = false;

  const uint32_t d0 = logGetUint(idDistance0);
  if (d0 > (DIST0_ABORT_MM + DIST0_HYST_MM)) {
    if (abortOverCount < 0xFF) abortOverCount++;
    if (abortOverCount >= ABORT_CONFIRM_COUNT) {
      if (!seqAbort) {
        DEBUG_PRINT("Emergency FAR: distance0=%lu mm (> %u+%u)\n",
                    (unsigned long)d0, DIST0_ABORT_MM, DIST0_HYST_MM);
      }
      seqAbort = true;
      trigger = true;
    }
  } else {
    abortOverCount = 0;
  }

  // NOTE: distance2 no longer triggers emergency land; handled by avoidance mode
  return trigger;
}

// Decide avoidance activation based on distance2 only (no derivative)
static bool updateAvoidanceMode(void) {
  const uint32_t d2 = logGetUint(idDistance2);

  // Immediate land if too close during avoidance
  if (avoidActive && d2 > 0 && d2 <= AVOID_MIN_LAND_MM) {
    DEBUG_PRINT("AVOID EMERGENCY LAND: d2=%lu mm <= %u mm\n",
                (unsigned long)d2, AVOID_MIN_LAND_MM);
    seqAbort = true;
    return true;
  }

  if (!avoidActive) {
    if (d2 > 0 && d2 <= DIST2_AVOID_MM) {
      if (++approachCount >= AVOID_ENTER_CONFIRM_COUNT) {
        avoidActive = true;
        approachCount = 0;
        departCount = 0;
        DEBUG_PRINT("AVOID start: d2=%lu mm\n", (unsigned long)d2);
      }
    } else {
      approachCount = 0;
    }
  } else {
    if (d2 >= (DIST2_AVOID_MM)) {
      if (++departCount >= AVOID_EXIT_CONFIRM_COUNT) {
        avoidActive = false;
        departCount = 0;
        approachCount = 0;
        DEBUG_PRINT("AVOID stop: d2=%lu mm\n", (unsigned long)d2);
      }
    } else {
      departCount = 0;
    }
  }

  return avoidActive;
}

static void landToZero(void) {
  // Descend at constant vertical velocity until near ground
  setpoint_t sp; memset(&sp, 0, sizeof(sp));
  sp.mode.x = modeVelocity; sp.velocity.x = 0;
  sp.mode.y = modeVelocity; sp.velocity.y = 0;
  sp.mode.z = modeVelocity; sp.velocity.z = -LAND_VZ_MPS;
  sp.mode.yaw = modeVelocity; sp.attitudeRate.yaw = 0;
  sp.velocity_body = true;
  while (1) {
    float z = logGetFloat(idZ);
    if (z >= 0.0f && z <= CUT_Z_M) break; // landing altitude reached
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  // Cut controllers and thrust to avoid bouncing
  setpoint_t cut; memset(&cut, 0, sizeof(cut));
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
    if (checkAndMaybeEmergencyLand()) return;
    const float z = (zTarget * (float)i) / (float)steps;
    sendHover(0.0f, 0.0f, z, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }
}

static void runSequence(void) {
  seqAbort = false;
  innerOverCount = innerUnderCount = 0;
  uint8_t routines = 0;  // count of (exit inner bound -> re-enter) cycles
  // Arc tracking/cooldown:
  // - arcActive: we are tracking rotation since inner exit while in TURN
  // - arcCooldown: once 270° arc is completed, do not re-trigger TURN until inner is re-entered
  bool arcActive = false;
  bool arcCooldown = false;
  float arcYawStart = 0.0f;
  float target_yaw = 0.0f;

  d0BufferReset();

  uint32_t startTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

  DEBUG_PRINT("Reactive: fwd, TURN if d0>=%u; AVOID if d2<=%u, CW yaw; land if d0>=%u; stop after certain time\n",
              INNER_BOUND_MM, DIST2_AVOID_MM, DIST0_ABORT_MM);

  // Takeoff
  rampToHeight(TARGET_HEIGHT_M, RAMP_TIME_MS);

  enum { STRAIGHT = 0, TURN = 1, RECOVER = 2 } mode = STRAIGHT;
  const uint32_t dtMs = 20;

  while (!seqAbort) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - startTime) > DEMO_TIME_MS) {
      break;
    }



    // Emergency checks (outer bound)
    if (checkAndMaybeEmergencyLand()) break;

    // Read distance0
    uint32_t d0 = logGetUint(idDistance0);

    // add distance0 to the moving average window for derivative calculation
    
    d0BufferAdd(d0, now);
    float d0Deriv = d0BufferGetDerivative();

    // DEBUG_PRINT("Current derivative: %.3f\n", (double)d0Deriv);

    // Clear arc cooldown once we re-enter the inner circle (with hysteresis)
    if (arcCooldown && d0 > 0 && d0 <= (INNER_BOUND_MM)) {
      arcCooldown = false;
      DEBUG_PRINT("ARC cooldown cleared by inner re-entry (d0=%lu)\n", (unsigned long)d0);
    }

    // Mode transitions + confirmation
    if (mode == STRAIGHT) {
      if (d0 >= (INNER_BOUND_MM)) {
        if (!arcCooldown && (++innerOverCount >= ABORT_CONFIRM_COUNT)) {
          // Exiting inner radius: enter TURN and start heading tracking
          mode = TURN;
          innerOverCount = 0;
          arcYawStart = logGetFloat(idYaw);
          target_yaw = normalizeAngle(arcYawStart - 90.0f); // 270 (-90) degrees from start
          arcActive = isfinite(arcYawStart);
          DEBUG_PRINT("ENTER TURN (d0=%lu), arcActive=%d, startYaw=%.3f deg\n",
                      (unsigned long)d0, arcActive ? 1 : 0, (double)arcYawStart);
        }
       } else {
         innerOverCount = 0;
       }
     } else { // TURN
      if (d0 <= (INNER_BOUND_MM)) {
        if (++innerUnderCount >= ABORT_CONFIRM_COUNT) {
          mode = STRAIGHT;
          innerUnderCount = 0;
          routines++;
          DEBUG_PRINT("EXIT TURN -> routine %u complete (d0=%lu)\n", routines, (unsigned long)d0);
        }
      } else {
        innerUnderCount = 0;
      }
     }

    // While TURN is active, accumulate rotation relative to arcYawStart.
    // When 270° reached (in the configured yaw direction), stop yaw by switching to STRAIGHT.
    if (mode == TURN && arcActive) {
      float curYaw = logGetFloat(idYaw);
      if (isfinite(curYaw)) {
        const bool reached = (fabsf(curYaw - target_yaw) <= 3.0f) || (fabsf(curYaw - target_yaw - 360.0f) <= 3.0f);
        if (reached) {
          DEBUG_PRINT("TURN arc reached 270° -> stop yaw\n");
          // Stop yaw by switching to STRAIGHT; keep forward velocity
          mode = STRAIGHT;
          arcActive = false;
          arcCooldown = true;  // do not re-trigger TURN until inner is re-entered
          innerUnderCount = 0; // avoid instant STRAIGHT->TURN flip-flop
          if (d0Deriv > 0) {
            DEBUG_PRINT("However, derivative was %.2f so going into recovery mode\n", (double)d0Deriv);
            mode = RECOVER;
          }
        }
      } else {
        // Lost yaw; stop arc tracking to avoid undefined behavior
        arcActive = false;
        DEBUG_PRINT("TURN arc: yaw unavailable, stopping arc tracking\n");
      }
    }

    // Avoidance overrides the normal command
    avoidWasActive = avoidActive;
    const bool avoid = updateAvoidanceMode();

    if ((avoidWasActive && !avoid) && (d0 >= INNER_BOUND_MM)){
      mode = RECOVER;
    }

    if (seqAbort) break;
    if (avoid) {
      sendHover(FWD_SPEED_MPS * AVOID_SPEED_FACTOR, 0.0f, TARGET_HEIGHT_M, AVOID_YAW_RATE_DPS); // CW
    } else if (mode == STRAIGHT) {
      // STRAIGHT: constant forward velocity, zero yaw
      sendHover(FWD_SPEED_MPS, 0.0f, TARGET_HEIGHT_M, 0.0f);
    } else if (mode == RECOVER) {
      // RECOVER: proportionally steer towards a negative derivative with mdidle beacon
      // desired yawrate = yawrate at zero / desired negative derivate * (abs(current deriv - target deriv))
      // but also a small deadzone around the negative derivatives that at least brings us closer and reduces the chance of overshoot
      float yawCommand = (50.0f/1400.0f) * fabsf(d0Deriv + 1400);
      yawCommand = yawCommand > 30.0f ? yawCommand : 0.0f;
      sendHover(FWD_SPEED_MPS, 0.0f, TARGET_HEIGHT_M, yawCommand);
      DEBUG_PRINT("Recovering with a yawrate of %.2f deg/s for a deriv of %.2f\n", (double)yawCommand, (double)d0Deriv);
      if (d0 > 0 && d0 <= (INNER_BOUND_MM)) {
        mode = STRAIGHT;
        DEBUG_PRINT("Made it back to the circle!\n");
      }
    } else
      {
      // TURN: constant forward velocity, configured yaw rate
      sendHover(FWD_SPEED_MPS, 0.0f, TARGET_HEIGHT_M, TURN_YAW_RATE_DPS);
    }

    vTaskDelay(pdMS_TO_TICKS(dtMs));
  }

  landToZero();
  if (seqAbort) {
    DEBUG_PRINT("Emergency landing\n");
  } else {
    landToZero();
    DEBUG_PRINT("Finished the demo in approx %u routines)\n", routines);
  }
}

void appMain(void) {
  DEBUG_PRINT("AUX0<%d triggers; inner=%u mm (turn), outer=%u mm (land), avoidance on d2<=%u (CW)\n",
              AUX_ACTIVE_THRESH, INNER_BOUND_MM, DIST0_ABORT_MM, DIST2_AVOID_MM);

  // Resolve required log IDs
  while (!logVarIdIsValid(idAux0) || 
    !logVarIdIsValid(idYaw)       ||
    !logVarIdIsValid(idDistance0) || 
    !logVarIdIsValid(idDistance2) ||
    !logVarIdIsValid(idZ) 
  ) {
    ensureLogId(&idAux0,      "cppm",    "aux0");
    ensureLogId(&idYaw, "stateEstimate", "yaw");
    ensureLogId(&idDistance0, "ranging", "distance0");
    ensureLogId(&idDistance2, "ranging", "distance2");
    ensureLogId(&idZ, "stateEstimate", "z");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  bool wasActive = false;
  while (1) {
    const bool active = aux0ActiveLow();

    // Emergency always active, even when idle
    if (checkAndMaybeEmergencyLand()) {
      landToZero();
    }

    if (active && !wasActive) {
      runSequence();
    }

    wasActive = active;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}