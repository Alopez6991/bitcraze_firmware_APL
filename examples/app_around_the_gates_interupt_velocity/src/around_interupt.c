/**
 * RC-switch triggered mission:
 *
 * STEP 1: TAKEOFF -> settle
 * STEP 2: Fly +Y 1.0 m
 * STEP 3: Fly +X 2.6 m
 * STEP 4: Fly -Y 7.1 m
 * STEP 5: Fly -X 2.6 m (only leg that can early-stop on spike within 0.2 m of target)
 *         Then creep further in -X and wait 10s before landing
 *
 * - Forces Kalman (stabilizer.estimator=2) and pulses kalman.resetEstimation
 * - WORLD-frame velocity (velocity_body=false), vz = 0.0
 * - Exponential smoothing + acceleration limiting
 * - Multiranger push-away on front/back/left/right
 * - Down-range spike detector (mm) with near-target gating (0.2 m)
 * - Trigger: cppm.aux3 < 1400 (edge-triggered; once per press)
 *
 * World axes convention: +X forward, +Y left, +Z up
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define DEBUG_MODULE "RCSEQ"
#include "debug.h"
#include "log.h"
#include "param.h"

#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"   // supervisorRequestArming(), supervisorIsArmed()

/* ---------------- User knobs ---------------- */
// Triggering / heights
#define AUX_ACTIVE_THRESH         1400        // cppm.aux3 < 1400 => active
#define TARGET_HEIGHT_M           0.75f
#define HOVER_TIME_MS             1000U
#define LAND_HOLD_MS              1500U
#define ARM_TIMEOUT_MS            2000U

// Velocity loop
#define LOOP_HZ                   50U         // control frequency
#define FEED_PERIOD_MS            (1000U/LOOP_HZ)
#define VEL_SMOOTH_TAU_S          0.12f       // exp smoothing time-constant (s)
#define MAX_ACCEL_MPS2            2.0f        // commanded accel limit

// Segment speeds
#define BASE_VEL_MPS              0.50f       // default segment speed
#define TAKEOFF_VEL_MPS           0.30f

// Segment distances
#define SEG2_POS_Y_M              1.0f        // +Y
#define SEG3_POS_X_M              2.6f        // +X
#define SEG4_NEG_Y_M              7.1f        // -Y
#define SEG5_NEG_X_M              2.6f        // -X (final leg)

// Multiranger avoidance
#define AVOID_MIN_DIST_M          0.20f       // start pushing if closer than this
#define AVOID_PUSH_VEL_MPS        0.50f       // per-axis push amount

// Down-range spike guard (units mm) with near-target gating at 0.2 m
#define STEP_THRESH_MM            90.0f      // step change threshold (mm)
#define RATE_THRESH_MM_PER_S      300.0f      // rate threshold (mm/s)
#define ENABLE_SPIKE_NEAR_DIST_M  0.20f       // only arm spike stop when this close to segment target

// Post-final-right (here final -X) creep and wait
#define CREEP_VEL_MPS             0.15f       // magnitude along -X
#define CREEP_TIME_MS             600U        // brief creep duration
#define WAIT_AFTER_FINAL_MS       10000U      // wait 10 seconds before landing

// Fallback for older trees (invalid is usually 0xFFFF)
#ifndef PARAM_VARID_IS_VALID
#define PARAM_VARID_IS_VALID(id) ((id) != (paramVarId_t)0xFFFF)
#endif

/* ---------------- Log IDs & flags ---------------- */
static logVarId_t idAux3  = 0xFFFFu;
static logVarId_t idX     = 0xFFFFu;
static logVarId_t idY     = 0xFFFFu;
static logVarId_t idFront = 0xFFFFu;
static logVarId_t idBack  = 0xFFFFu;
static logVarId_t idLeft  = 0xFFFFu;
static logVarId_t idRight = 0xFFFFu;
static logVarId_t idDown  = 0xFFFFu;

static bool isActive = false;
static bool sequenceDoneUntilReset = false;

/*********** Helpers: setpoint & timing ***********/
static void setHoverSetpoint(setpoint_t* sp, float vx, float vy, float zAbs, float yawRate, bool worldFrame)
{
  memset(sp, 0, sizeof(*sp));

  sp->mode.z         = modeAbs;
  sp->position.z     = zAbs;

  sp->mode.x         = modeVelocity;
  sp->mode.y         = modeVelocity;
  sp->velocity.x     = vx;
  sp->velocity.y     = vy;

  sp->mode.yaw       = modeVelocity;
  sp->attitudeRate.yaw = yawRate;

  sp->velocity_body  = !worldFrame;   // false => WORLD frame
}

static void holdZ_ms(float zAbs, uint32_t ms)
{
  const TickType_t step = pdMS_TO_TICKS(FEED_PERIOD_MS);
  TickType_t left = pdMS_TO_TICKS(ms);
  setpoint_t sp;

  while (left > 0) {
    setHoverSetpoint(&sp, 0.0f, 0.0f, zAbs, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(step);
    left = (left > step) ? (left - step) : 0;
  }
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

static void zeroBurst_ms(uint32_t ms, uint32_t hz)
{
  const TickType_t step = pdMS_TO_TICKS(1000U / (hz ? hz : 100U));
  TickType_t left = pdMS_TO_TICKS(ms);
  setpoint_t sp;

  while (left > 0) {
    setHoverSetpoint(&sp, 0.0f, 0.0f, TARGET_HEIGHT_M, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(step);
    left = (left > step) ? (left - step) : 0;
  }
}

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

// Smooth vertical climb at vz_takeoff (m/s) to zTarget (m)
static void rampedTakeoff(float zTarget, float vz_takeoff) {
  const float dt = 1.0f / (float)LOOP_HZ;
  float z = 0.0f;
  setpoint_t sp;

  if (vz_takeoff < 0.05f) vz_takeoff = 0.05f;      // avoid super-slow rise
  if (vz_takeoff > 0.6f)  vz_takeoff = 0.6f;       // keep gentle

  while (z < zTarget) {
    z += vz_takeoff * dt;
    if (z > zTarget) z = zTarget;

    setHoverSetpoint(&sp, 0.0f, 0.0f, z, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(pdMS_TO_TICKS(FEED_PERIOD_MS));
  }
}

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

/*********** Avoid helper ***********/
static inline bool is_close_m(float m) {
  return isfinite(m) && (m < AVOID_MIN_DIST_M);
}

/*********** Generic segment with spike detection (forward or lateral) ***********/
static bool runVelAlongDir(float speed, float targetDist, float dirX, float dirY, bool stopOnSpike)
{
  // Normalize direction
  const float norm = sqrtf(dirX*dirX + dirY*dirY);
  if (norm < 1e-6f) return false;
  const float ux = dirX / norm;
  const float uy = dirY / norm;

  // Ensure we have state log IDs
  if (!logVarIdIsValid(idX)) idX = logGetVarId("stateEstimate", "x");
  if (!logVarIdIsValid(idY)) idY = logGetVarId("stateEstimate", "y");

  // Range sides
  if (!logVarIdIsValid(idFront)) idFront = logGetVarId("range", "front");
  if (!logVarIdIsValid(idBack))  idBack  = logGetVarId("range", "back");
  if (!logVarIdIsValid(idLeft))  idLeft  = logGetVarId("range", "left");
  if (!logVarIdIsValid(idRight)) idRight = logGetVarId("range", "right");

  // Downward range: try known (group,name) pairs (reuse idDown if already resolved)
  if (!logVarIdIsValid(idDown)) {
    struct { const char* g; const char* n; } cands[] = {
      { "range", "zrange" }, { "range", "zrangeMm" }, { "range", "zDistance" },
      { "range", "down"   }, { "range", "altitude" }, { "range", "distance" },
    };
    for (size_t i = 0; i < sizeof(cands)/sizeof(cands[0]); ++i) {
      logVarId_t id = logGetVarId(cands[i].g, cands[i].n);
      if (logVarIdIsValid(id)) { idDown = id; break; }
    }
  }

  // Wait until x,y are valid
  float x = logGetFloat(idX), y = logGetFloat(idY);
  while (!isfinite(x) || !isfinite(y)) {
    vTaskDelay(pdMS_TO_TICKS(10));
    x = logGetFloat(idX); y = logGetFloat(idY);
  }
  const float x0 = x, y0 = y;

  const TickType_t loopStep = pdMS_TO_TICKS(FEED_PERIOD_MS);
  const float dt = 1.0f / (float)LOOP_HZ;
  const float alpha = dt / (VEL_SMOOTH_TAU_S + dt);
  const float dv_max = MAX_ACCEL_MPS2 * dt;

  setpoint_t sp;
  float sm_vx = ux * speed, sm_vy = uy * speed;
  float sent_vx = sm_vx,   sent_vy = sm_vy;

  // Spike detector history (down range is typically in mm)
  bool spiked = false;
  float lastDown_mm = NAN;
  TickType_t lastDownTick = 0;

  for (;;) {
    // Signed progress along (ux,uy)
    x = logGetFloat(idX); y = logGetFloat(idY);
    if (isfinite(x) && isfinite(y)) {
      const float dx = x - x0;
      const float dy = y - y0;
      const float s = dx*ux + dy*uy;
      if (s >= targetDist) break;
    }

    // Spike detection active near target; stop only if stopOnSpike==true
    if (logVarIdIsValid(idDown)) {
      float r_mm = logGetFloat(idDown);
      TickType_t now = xTaskGetTickCount();
      if (isfinite(r_mm)) {
        if (isfinite(lastDown_mm)) {
          float dr = r_mm - lastDown_mm;                                   // mm
          float dt_s = (float)(now - lastDownTick) / (float)configTICK_RATE_HZ;
          if (dt_s < 1e-3f) dt_s = 1e-3f;
          float rate = dr / dt_s;                                          // mm/s

          // Remaining distance estimate along path
          const float dx = x - x0, dy = y - y0;
          const float s  = dx*ux + dy*uy;
          const float remaining = targetDist - s;

          const bool step_hit = (dr <= -STEP_THRESH_MM);
          const bool rate_hit = (fabsf(rate) >= RATE_THRESH_MM_PER_S);
          if ((remaining <= ENABLE_SPIKE_NEAR_DIST_M) && step_hit && rate_hit) {
            DEBUG_PRINT("[SPIKE-%s] dR=%.1f mm, rate=%.1f mm/s, rem=%.2f m\n",
                        stopOnSpike ? "STOP" : "LOG",
                        (double)dr, (double)rate, (double)remaining);
            if (stopOnSpike) {
              spiked = true;
              // Brake once, then exit
              setHoverSetpoint(&sp, 0.0f, 0.0f, TARGET_HEIGHT_M, 0.0f, true);
              commanderSetSetpoint(&sp, 3);
              break;
            }
          }
        }
        lastDown_mm = r_mm;
        lastDownTick = now;
      }
    }

    // Base desired vel along path
    float vx_des = ux * speed;
    float vy_des = uy * speed;

    // Multiranger push-away
    float f = logGetFloat(idFront);
    float b = logGetFloat(idBack);
    float l = logGetFloat(idLeft);
    float r = logGetFloat(idRight);
    if (is_close_m(f)) vx_des -= AVOID_PUSH_VEL_MPS;
    if (is_close_m(b)) vx_des += AVOID_PUSH_VEL_MPS;
    if (is_close_m(l)) vy_des -= AVOID_PUSH_VEL_MPS;
    if (is_close_m(r)) vy_des += AVOID_PUSH_VEL_MPS;

    // Limit to a sane max
    float vmax = fmaxf(fabsf(speed), AVOID_PUSH_VEL_MPS);
    if (vx_des >  vmax) vx_des =  vmax;
    if (vx_des < -vmax) vx_des = -vmax;
    if (vy_des >  vmax) vy_des =  vmax;
    if (vy_des < -vmax) vy_des = -vmax;

    // Smooth + accel limit
    sm_vx += alpha * (vx_des - sm_vx);
    sm_vy += alpha * (vy_des - sm_vy);
    float dvx = sm_vx - sent_vx;
    float dvy = sm_vy - sent_vy;
    if (dvx >  dv_max) dvx =  dv_max; else if (dvx < -dv_max) dvx = -dv_max;
    if (dvy >  dv_max) dvy =  dv_max; else if (dvy < -dv_max) dvy = -dv_max;
    const float vx_cmd = sent_vx + dvx;
    const float vy_cmd = sent_vy + dvy;
    sent_vx = vx_cmd; sent_vy = vy_cmd;

    // Send WORLD-frame velocity, keep altitude
    setHoverSetpoint(&sp, vx_cmd, vy_cmd, TARGET_HEIGHT_M, 0.0f, true);
    commanderSetSetpoint(&sp, 3);

    vTaskDelay(loopStep);
  }

  // Stop cleanly
  zeroBurst_ms(/*ms*/300, /*hz*/100);
  return spiked;
}

/*********** Full sequence ***********/
static void runSequence(void)
{
  DEBUG_PRINT("Sequence: ARM -> TAKEOFF -> settle -> +Y -> +X -> -Y -> -X (spike-gated) -> creep -X & wait -> LAND -> DISARM\n");

  // STEP 1: TAKEOFF and settle
  forceKalmanAndReset();

  supervisorRequestArming(true);
  if (!waitForArmed(ARM_TIMEOUT_MS)) {
    DEBUG_PRINT("WARN: did not report ARMED within %u ms, continuing\n", (unsigned)ARM_TIMEOUT_MS);
  }

  // Neutral warmup on ground
  holdZ_ms(0.0f, 800U);

  // Smooth takeoff to target height
  rampedTakeoff(TARGET_HEIGHT_M, TAKEOFF_VEL_MPS);

  // Settle
  holdZ_ms(TARGET_HEIGHT_M, 1500U);
  holdZ_ms(TARGET_HEIGHT_M, HOVER_TIME_MS);

  // STEP 2: Fly +Y by 1.0 m
  (void)runVelAlongDir(/*speed*/BASE_VEL_MPS, /*dist*/SEG2_POS_Y_M, /*dirX*/0.0f, /*dirY*/+1.0f, /*stopOnSpike*/false);
  holdZ_ms(TARGET_HEIGHT_M, 300U);

  // STEP 3: Fly +X by 2.6 m
  (void)runVelAlongDir(/*speed*/BASE_VEL_MPS, /*dist*/SEG3_POS_X_M, /*dirX*/+1.0f, /*dirY*/0.0f, /*stopOnSpike*/false);
  holdZ_ms(TARGET_HEIGHT_M, 300U);

  // STEP 4: Fly -Y by 7.1 m
  (void)runVelAlongDir(/*speed*/BASE_VEL_MPS, /*dist*/SEG4_NEG_Y_M, /*dirX*/0.0f, /*dirY*/-1.0f, /*stopOnSpike*/false);
  holdZ_ms(TARGET_HEIGHT_M, 300U);

  // STEP 5: Fly -X by 2.6 m with spike stop only if within 0.2 m of target
  (void)runVelAlongDir(/*speed*/BASE_VEL_MPS, /*dist*/SEG5_NEG_X_M, /*dirX*/-1.0f, /*dirY*/0.0f, /*stopOnSpike*/true);
  holdZ_ms(TARGET_HEIGHT_M, HOVER_TIME_MS);

  // After STEP 5: creep further in -X and wait 10 seconds
  feedHover_ms(/*vx*/-CREEP_VEL_MPS, /*vy*/0.0f, TARGET_HEIGHT_M, CREEP_TIME_MS);
  zeroBurst_ms(/*ms*/300, /*hz*/100);
  holdZ_ms(TARGET_HEIGHT_M, WAIT_AFTER_FINAL_MS);

  // LAND & DISARM after 10s wait
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

    const int16_t aux3 = logGetInt(idAux3);
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
