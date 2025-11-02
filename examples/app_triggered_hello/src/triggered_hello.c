/**
 * RC-switch triggered: ARM -> TAKEOFF -> settle -> WORLD-vel loop to distance -> HOVER -> LAND -> DISARM
 * - Uses Kalman estimator (forces stabilizer.estimator=2 and kalman.resetEstimation)
 * - WORLD-frame velocity (velocity_body = false), vz = 0.0
 * - Exponential smoothing + acceleration limiting (like Python)
 * - Trigger: cppm.aux3 < 1400 (once per press)
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
#define AUX_ACTIVE_THRESH       1400       // cppm.aux3 < 1400 => active
#define TARGET_HEIGHT_M         1.00f
#define HOVER_TIME_MS           2000U
#define LAND_HOLD_MS            2500U

#define LOOP_HZ                 50U        // continuous velocity loop rate (50–100)
#define FEED_PERIOD_MS          (1000U/LOOP_HZ)

#define BASE_FWD_VEL            0.50f      // m/s world +X
#define TARGET_DISTANCE_M       2.50f      // stop after this distance (from start)
#define VEL_SMOOTH_TAU_S        0.12f      // exp smoothing time-constant (s)
#define MAX_ACCEL_MPS2          2.0f       // rate-limit (m/s^2)

#define ARM_TIMEOUT_MS          2000U
/* -------------------------------------------- */

static logVarId_t idAux3 = 0xffffu;
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

// Feed neutral (vx=vy=0, zAbs stays at current) for ms milliseconds
static void warmupAfterArm_ms(uint32_t ms) {
  const TickType_t step = pdMS_TO_TICKS(FEED_PERIOD_MS);
  TickType_t left = pdMS_TO_TICKS(ms);
  setpoint_t sp;

  while (left > 0) {
    setHoverSetpoint(&sp, 0.0f, 0.0f, 0.0f, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(step);
    left = (left > step) ? (left - step) : 0;
  }
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

/*********** Distance source: stateEstimate.{x,y} ***********/
static logVarId_t idX = 0xffffu, idY = 0xffffu;
static float getX(void) { return logVarIdIsValid(idX) ? logGetFloat(idX) : NAN; }
static float getY(void) { return logVarIdIsValid(idY) ? logGetFloat(idY) : NAN; }

/*********** Continuous world-velocity loop ***********/
static void runWorldVelToDistance(float baseVx, float targetDist)
{
  // Ensure we have state log IDs
  if (!logVarIdIsValid(idX)) idX = logGetVarId("stateEstimate", "x");
  if (!logVarIdIsValid(idY)) idY = logGetVarId("stateEstimate", "y");

  // Wait until x,y are valid
  float x = getX(), y = getY();
  while (!isfinite(x) || !isfinite(y)) {
    vTaskDelay(pdMS_TO_TICKS(10));
    x = getX(); y = getY();
  }
  const float x0 = x, y0 = y;

  const TickType_t step = pdMS_TO_TICKS(FEED_PERIOD_MS);
  setpoint_t sp;

  float sm_vx = baseVx, sm_vy = 0.0f;
  float sent_vx = baseVx, sent_vy = 0.0f;

  const float dt = 1.0f / (float)LOOP_HZ;
  const float alpha = dt / (VEL_SMOOTH_TAU_S + dt);
  const float dv_max = MAX_ACCEL_MPS2 * dt;

  for (;;) {
    // Distance check
    x = getX(); y = getY();
    if (isfinite(x) && isfinite(y)) {
      const float dx = x - x0;
      const float dy = y - y0;
      const float dist = sqrtf(dx*dx + dy*dy);
      if (dist >= targetDist) break;
    }

    // Desired world-frame velocity (straight +X)
    float vx_des = baseVx;
    float vy_des = 0.0f;

    // Exponential smoothing
    sm_vx = sm_vx + alpha * (vx_des - sm_vx);
    sm_vy = sm_vy + alpha * (vy_des - sm_vy);

    // Acceleration limiting
    const float dvx = fmaxf(-dv_max, fminf(dv_max, sm_vx - sent_vx));
    const float dvy = fmaxf(-dv_max, fminf(dv_max, sm_vy - sent_vy));
    const float vx_cmd = sent_vx + dvx;
    const float vy_cmd = sent_vy + dvy;
    sent_vx = vx_cmd;
    sent_vy = vy_cmd;

    // Send WORLD-frame velocity, keep altitude
    setHoverSetpoint(&sp, vx_cmd, vy_cmd, TARGET_HEIGHT_M, 0.0f, true);
    commanderSetSetpoint(&sp, 3);

    vTaskDelay(step);
  }

  // Stop cleanly with a short burst of zeros
  for (int i = 0; i < 30; ++i) {
    setHoverSetpoint(&sp, 0.0f, 0.0f, TARGET_HEIGHT_M, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/*********** Full sequence ***********/
static void runSequence(void)
{
  DEBUG_PRINT("Sequence: ARM -> TAKEOFF -> settle -> vel-loop -> HOVER -> LAND -> DISARM\n");

  // Force Kalman + reset (like your Python)
  forceKalmanAndReset();

  // ARM
  supervisorRequestArming(true);
  if (!waitForArmed(ARM_TIMEOUT_MS)) {
    DEBUG_PRINT("WARN: did not report ARMED within %ums, continuing\n", ARM_TIMEOUT_MS);
  }

  // Motor/ESC + EKF warmup: neutral feed on ground
  warmupAfterArm_ms(800);                   // 0.8 s neutral

  // Smooth takeoff to target height
  rampedTakeoff(TARGET_HEIGHT_M, /*vz*/0.30f);

  // Takeoff & settle around target height
  holdZ_ms(TARGET_HEIGHT_M, 1500U);
  holdZ_ms(TARGET_HEIGHT_M, HOVER_TIME_MS);

  // Continuous WORLD-frame velocity loop to distance
  runWorldVelToDistance(BASE_FWD_VEL, TARGET_DISTANCE_M);

  // Post-stop hover
  holdZ_ms(TARGET_HEIGHT_M, HOVER_TIME_MS);

  // Land & disarm
  holdZ_ms(0.0f, LAND_HOLD_MS);
  // neutral
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
