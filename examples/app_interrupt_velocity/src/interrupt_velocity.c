/**
 * RC-switch triggered: ARM -> TAKEOFF -> settle -> WORLD-vel loop to distance/guards -> HOVER -> creep fwd -> LAND -> DISARM
 * - Forces Kalman (stabilizer.estimator=2) and pulses kalman.resetEstimation
 * - WORLD-frame velocity (velocity_body=false), vz = 0.0
 * - Exponential smoothing + acceleration limiting
 * - Multiranger-style push-away on front/back/left/right (no UP)
 * - Down-range spike detector to stop early (terrain rise / edge)
 * - After stop: zero-burst, brief hover, creep forward a bit, land
 * - Trigger: cppm.aux3 < 1400 (edge-triggered; once per press)
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#define DEBUG_MODULE "RCSEQ"
#include "log.h"
#include "param.h"

#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"   // supervisorRequestArming(), supervisorIsArmed()

/* ---------------- User knobs ---------------- */
// Triggering / heights
#define AUX_ACTIVE_THRESH         1400        // cppm.aux3 < 1400 => active
#define TARGET_HEIGHT_M           1.00f
#define HOVER_TIME_MS             2000U
#define LAND_HOLD_MS              1500U
#define ARM_TIMEOUT_MS            2000U

// Velocity loop
#define LOOP_HZ                   50U         // 50–100 Hz is fine
#define FEED_PERIOD_MS            (1000U/LOOP_HZ)
#define BASE_FWD_VEL              0.50f       // m/s world +X
#define TARGET_DISTANCE_M         3.50f       // nominal stop distance from start
#define VEL_SMOOTH_TAU_S          0.12f       // exp smoothing time-constant (s)
#define MAX_ACCEL_MPS2            2.0f        // commanded accel limit

// Takeoff/land profiles
#define TAKEOFF_VEL_MPS           0.30f
#define LAND_VEL_MPS              0.30f

// Multiranger push-away (no 'up')
#define AVOID_MIN_DIST_M          0.20f       // start pushing if closer than this
#define AVOID_PUSH_VEL_MPS        0.50f       // per-axis push amount

// Simple C helper for distance check
static inline bool is_close_m(float m) {
  return isfinite(m) && (m < AVOID_MIN_DIST_M);
}

// Down-range spike detector (terrain rise / edge)
#define MONITOR_PERIOD_MS         20U         // ~50 Hz
#define STEP_THRESH_MM            20.0f       // spike if step <= -20 mm
#define RATE_THRESH_MM_PER_S      300.0f      // or |rate| >= 300 mm/s
#define ENABLE_SPIKE_NEAR_DIST_M  0.25f       // only allow spike-stop when within this remaining distance
#define MOVE_FORWARD_AFTER_EDGE_M 0.55f       // creep forward after spike stop
#define CREEP_VEL_MPS             0.10f       // creep velocity

/* -------------------------------------------- */

static logVarId_t idAux3 = 0xffffu;
static bool isActive = false;
static bool sequenceDoneUntilReset = false;

/*********** Helpers: setpoint & timing ***********/
static void setHoverSetpoint(setpoint_t* sp, float vx, float vy, float zAbs, float yawRate, bool worldFrame)
{
  memset(sp, 0, sizeof(*sp));

  sp->mode.z            = modeAbs;
  sp->position.z        = zAbs;

  sp->mode.x            = modeVelocity;
  sp->mode.y            = modeVelocity;
  sp->velocity.x        = vx;
  sp->velocity.y        = vy;

  sp->mode.yaw          = modeVelocity;
  sp->attitudeRate.yaw  = yawRate;

  sp->velocity_body     = !worldFrame;   // false => WORLD frame
}

static void feedHover_ms(float vx, float vy, float zAbs, uint32_t ms)
{
  const TickType_t step = pdMS_TO_TICKS(FEED_PERIOD_MS);
  TickType_t left = pdMS_TO_TICKS(ms);
  setpoint_t sp;

  while (left > 0) {
    setHoverSetpoint(&sp, vx, vy, zAbs, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(step);
    left = (left > step) ? (left - step) : 0;
  }
}

static void holdZ_ms(float zAbs, uint32_t ms) { feedHover_ms(0.0f, 0.0f, zAbs, ms); }

static bool waitForArmed(uint32_t timeout_ms)
{
  const TickType_t t0 = xTaskGetTickCount();
  const TickType_t dt = pdMS_TO_TICKS(10);
  const TickType_t to = pdMS_TO_TICKS(timeout_ms);

  while (!supervisorIsArmed()) {
    vTaskDelay(dt);
    if ((xTaskGetTickCount() - t0) > to) return false;
  }
  return true;
}

// After a stop, spam zero-world-vel so the halt "sticks"
static void zeroBurst_ms(uint32_t ms, uint32_t hz)
{
  uint32_t period_ms = (hz > 0) ? (1000U / hz) : 10U;
  const TickType_t step = pdMS_TO_TICKS(period_ms);
  TickType_t left = pdMS_TO_TICKS(ms);
  setpoint_t sp;
  while (left > 0) {
    setHoverSetpoint(&sp, 0.0f, 0.0f, TARGET_HEIGHT_M, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(step);
    left = (left > step) ? (left - step) : 0;
  }
}

// Smooth vertical climb at vz_takeoff (m/s) to zTarget (m)
static void rampedTakeoff(float zTarget, float vz_takeoff)
{
  const float dt = 1.0f / (float)LOOP_HZ;
  float z = 0.0f;
  setpoint_t sp;

  if (vz_takeoff < 0.05f) vz_takeoff = 0.05f;
  if (vz_takeoff > 0.6f)  vz_takeoff = 0.6f;

  while (z < zTarget) {
    z += vz_takeoff * dt;
    if (z > zTarget) z = zTarget;

    setHoverSetpoint(&sp, 0.0f, 0.0f, z, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(pdMS_TO_TICKS(FEED_PERIOD_MS));
  }
}

// Fallback for older trees (invalid is usually 0xFFFF)
#ifndef PARAM_VARID_IS_VALID
#define PARAM_VARID_IS_VALID(id) ((id) != (paramVarId_t)0xFFFF)
#endif

/*********** Param helpers: force Kalman ***********/
static void forceKalmanAndReset(void)
{
  const paramVarId_t idEst   = paramGetVarId("stabilizer", "estimator");
  const paramVarId_t idReset = paramGetVarId("kalman", "resetEstimation");

  if (PARAM_VARID_IS_VALID(idEst)) {
    paramSetInt(idEst, 2);  // 2 = Kalman
  } else {
    DEBUG_PRINT("Param not found: stabilizer.estimator\n");
  }

  if (PARAM_VARID_IS_VALID(idReset)) {
    // pulse resetEstimation 1 -> 0 to reinit the EKF
    paramSetInt(idReset, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    paramSetInt(idReset, 0);
  } else {
    DEBUG_PRINT("Param not found: kalman.resetEstimation\n");
  }
}

/*********** Distance & ranges via LOG API ***********/
static logVarId_t idX = 0xffffu, idY = 0xffffu;

// Multiranger sides (meters)
static logVarId_t idFront = 0xffffu, idBack = 0xffffu, idLeft = 0xffffu, idRight = 0xffffu;

// Downward range: try multiple common names (first that exists is used)
static const char* DOWN_CANDIDATES[] __attribute__((unused)) = {
  "range.zrange", "range.zDistance", "range.down", "range.altitude", "range.distance"
};
static logVarId_t idDown = 0xffffu;
static const char* downName = NULL;

static float getFloat(logVarId_t id) { return logVarIdIsValid(id) ? logGetFloat(id) : NAN; }
static int16_t getInt(logVarId_t id) { return logVarIdIsValid(id) ? logGetInt(id) : 0; }

/*********** Continuous world-velocity loop with avoidance & spike guard ***********/
static void runWorldVelToDistance(float baseVx, float targetDist)
{
  // Ensure we have state log IDs
  if (!logVarIdIsValid(idX)) idX = logGetVarId("stateEstimate", "x");
  if (!logVarIdIsValid(idY)) idY = logGetVarId("stateEstimate", "y");

  // Range sides
  if (!logVarIdIsValid(idFront)) idFront = logGetVarId("range", "front");
  if (!logVarIdIsValid(idBack))  idBack  = logGetVarId("range", "back");
  if (!logVarIdIsValid(idLeft))  idLeft  = logGetVarId("range", "left");
  if (!logVarIdIsValid(idRight)) idRight = logGetVarId("range", "right");

  // Downward range: try known (group,name) pairs
  if (!logVarIdIsValid(idDown)) {
    struct { const char* g; const char* n; } cands[] = {
      { "range", "zrange" },
      { "range", "zrangeMm" },
      { "range", "zDistance" },
      { "range", "down" },
      { "range", "altitude" },
      { "range", "distance" },
    };
    for (size_t i = 0; i < sizeof(cands)/sizeof(cands[0]); ++i) {
      logVarId_t id = logGetVarId(cands[i].g, cands[i].n);
      if (logVarIdIsValid(id)) { idDown = id; downName = cands[i].g; /* keep name separately */ break; }
    }
    if (logVarIdIsValid(idDown)) {
      DEBUG_PRINT("[monitor] Down-range via '%s.%s'\n", downName ? downName : "range", "z*");
    } else {
      DEBUG_PRINT("[monitor] No down-range var found; spike guard disabled.\n");
    }
  }

  // Wait until x,y are valid
  float x = getFloat(idX), y = getFloat(idY);
  while (!isfinite(x) || !isfinite(y)) {
    vTaskDelay(pdMS_TO_TICKS(10));
    x = getFloat(idX); y = getFloat(idY);
  }
  const float x0 = x, y0 = y;

  const TickType_t loopStep = pdMS_TO_TICKS(FEED_PERIOD_MS);
  const float dt = 1.0f / (float)LOOP_HZ;
  const float alpha = dt / (VEL_SMOOTH_TAU_S + dt);
  const float dv_max = MAX_ACCEL_MPS2 * dt;

  setpoint_t sp;
  float sm_vx = baseVx, sm_vy = 0.0f;
  float sent_vx = baseVx, sent_vy = 0.0f;

  // Spike detector history (down range is typically in mm)
  bool spikeAbort = false;
  float lastDown_mm = NAN;
  TickType_t lastDownTick = 0;

  DEBUG_PRINT("Velocity-control loop started …\n");

  for (;;) {
    // Distance check
    x = getFloat(idX); y = getFloat(idY);
    float dist = 0.0f;
    if (isfinite(x) && isfinite(y)) {
      const float dx = x - x0;
      const float dy = y - y0;
      dist = sqrtf(dx*dx + dy*dy);
      if (dist >= targetDist) {
        DEBUG_PRINT("Target distance reached: %.2f m\n", (double)dist);
        break;
      }
    }

    // Spike detection (enable only when close to target to avoid early false stops)
    if (!spikeAbort && logVarIdIsValid(idDown) && isfinite(x)) {
      float r_mm = getFloat(idDown); // in mm in most firmwares; if meters, thresholds are small but still safe
      TickType_t now = xTaskGetTickCount();
      if (isfinite(r_mm)) {
        if (isfinite(lastDown_mm)) {
          float dr = r_mm - lastDown_mm;                                   // mm
          float dt_s = (float)(now - lastDownTick) / (float)configTICK_RATE_HZ;
          if (dt_s < 1e-3f) dt_s = 1e-3f;
          float rate = dr / dt_s;                                          // mm/s
          float remaining = targetDist - dist;
          bool step_hit = (dr <= -STEP_THRESH_MM);
          bool rate_hit = (fabsf(rate) >= RATE_THRESH_MM_PER_S);
          if ((remaining <= ENABLE_SPIKE_NEAR_DIST_M) && step_hit && rate_hit) {
            spikeAbort = true;
            DEBUG_PRINT("[SPIKE] dR=%.1f mm, rate=%.1f mm/s, remaining=%.2f m\n",
                        (double)dr, (double)rate, (double)remaining);
          }
        }
        lastDown_mm = r_mm;
        lastDownTick = now;
      }
    }

    // Base desired velocities
    float vx_des = baseVx;
    float vy_des = 0.0f;

    // Multiranger push-away
    float f = getFloat(idFront);
    float b = getFloat(idBack);
    float l = getFloat(idLeft);
    float r = getFloat(idRight);

    if (is_close_m(f)) vx_des -= AVOID_PUSH_VEL_MPS;
    if (is_close_m(b)) vx_des += AVOID_PUSH_VEL_MPS;
    if (is_close_m(l)) vy_des -= AVOID_PUSH_VEL_MPS;
    if (is_close_m(r)) vy_des += AVOID_PUSH_VEL_MPS;

    // clamp to sane max (use the larger of base or push)
    float vmax = (BASE_FWD_VEL > AVOID_PUSH_VEL_MPS) ? BASE_FWD_VEL : AVOID_PUSH_VEL_MPS;
    if (vx_des >  vmax) vx_des =  vmax;
    if (vx_des < -vmax) vx_des = -vmax;
    if (vy_des >  vmax) vy_des =  vmax;
    if (vy_des < -vmax) vy_des = -vmax;

    // Exponential smoothing
    sm_vx = sm_vx + alpha * (vx_des - sm_vx);
    sm_vy = sm_vy + alpha * (vy_des - sm_vy);

    // Acceleration limiting
    float dvx = sm_vx - sent_vx;
    float dvy = sm_vy - sent_vy;
    if (dvx >  dv_max) dvx =  dv_max; else if (dvx < -dv_max) dvx = -dv_max;
    if (dvy >  dv_max) dvy =  dv_max; else if (dvy < -dv_max) dvy = -dv_max;
    const float vx_cmd = sent_vx + dvx;
    const float vy_cmd = sent_vy + dvy;
    sent_vx = vx_cmd;
    sent_vy = vy_cmd;

    // If spikeAbort flipped, break after pushing one more zeroed cycle below
    if (spikeAbort) {
      // Send one zero immediately to start braking feel
      setHoverSetpoint(&sp, 0.0f, 0.0f, TARGET_HEIGHT_M, 0.0f, true);
      commanderSetSetpoint(&sp, 3);
      break;
    }

    // Send WORLD-frame velocity, keep altitude
    setHoverSetpoint(&sp, vx_cmd, vy_cmd, TARGET_HEIGHT_M, 0.0f, true);
    commanderSetSetpoint(&sp, 3);

    vTaskDelay(loopStep);
  }

  // Stop cleanly
  zeroBurst_ms(/*ms*/400, /*hz*/100);

  // Brief hover
  holdZ_ms(TARGET_HEIGHT_M, HOVER_TIME_MS);

  // If we stopped because of a spike, creep forward a bit to avoid clipping the edge
  if (true) { // creep regardless—harmless after distance stop, helpful after spike
    const float creepTime_s = MOVE_FORWARD_AFTER_EDGE_M / (CREEP_VEL_MPS > 0.05f ? CREEP_VEL_MPS : 0.05f);
    const uint32_t creep_ms = (uint32_t)(creepTime_s * 1000.0f);
    feedHover_ms(CREEP_VEL_MPS, 0.0f, TARGET_HEIGHT_M, creep_ms);
    zeroBurst_ms(/*ms*/300, /*hz*/100);
  }
}

/*********** Full sequence ***********/
static void runSequence(void)
{
  DEBUG_PRINT("Sequence: ARM -> TAKEOFF -> settle -> vel-loop -> HOVER/creep -> LAND -> DISARM\n");

  // Force Kalman + reset
  forceKalmanAndReset();

  // ARM
  supervisorRequestArming(true);
  if (!waitForArmed(ARM_TIMEOUT_MS)) {
    DEBUG_PRINT("WARN: did not report ARMED within %ums, continuing\n", ARM_TIMEOUT_MS);
  }

  // Neutral warmup
  holdZ_ms(0.0f, 800U);

  // Smooth takeoff to target height
  rampedTakeoff(TARGET_HEIGHT_M, TAKEOFF_VEL_MPS);

  // Settle
  holdZ_ms(TARGET_HEIGHT_M, 1500U);
  holdZ_ms(TARGET_HEIGHT_M, HOVER_TIME_MS);

  // WORLD-frame velocity loop (with avoidance + spike guard)
  runWorldVelToDistance(BASE_FWD_VEL, TARGET_DISTANCE_M);

  // Post-stop hover
  holdZ_ms(TARGET_HEIGHT_M, HOVER_TIME_MS);

  // Land & disarm (gentle)
  // Descend by commanding zAbs towards 0 with a hold so estimator stays happy
  holdZ_ms(0.0f, LAND_HOLD_MS);
  holdZ_ms(0.0f, 200U);
  supervisorRequestArming(false);

  DEBUG_PRINT("Sequence complete\n");
}

/*********** Main: switch edge detection ***********/
void appMain(void)
{
  DEBUG_PRINT("Waiting for activation ...\n");

  TickType_t lastPrint = 0;

  while (1) {
    vTaskDelay(F2T(50));   // 50 Hz poll

    if (!logVarIdIsValid(idAux3)) {
      idAux3 = logGetVarId("cppm", "aux3");
      continue;
    }

    const int16_t aux3 = getInt(idAux3);
    const bool nowActive = (aux3 > 0) && (aux3 < AUX_ACTIVE_THRESH);

    if (nowActive && !isActive) {
      DEBUG_PRINT("Switch activated\n");
      if (!sequenceDoneUntilReset) {
        runSequence();
        sequenceDoneUntilReset = true;
      }
      lastPrint = xTaskGetTickCount();
      isActive = true;
      continue;
    }

    if (!nowActive && isActive) {
      DEBUG_PRINT("Switch released (re-armed)\n");
      isActive = false;
      sequenceDoneUntilReset = false;
      continue;
    }

    if (isActive && (xTaskGetTickCount() - lastPrint) >= M2T(2000)) {
      DEBUG_PRINT("Switch still active…\n");
      lastPrint = xTaskGetTickCount();
    }
  }
}
