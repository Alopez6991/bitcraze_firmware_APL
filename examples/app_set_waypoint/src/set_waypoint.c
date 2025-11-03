/**
 * Simple waypoint runner:
 * - Takeoff to z1 (same sequence as interrupt_velocity.c)
 * - Move to p1 at v1
 * - Move to p2 at v1
 * - Change altitude to z2 at p2
 * - Move to p3 at v2
 * - Land
 *
 * Small hover between legs.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "WP"
#include "debug.h"
#include "log.h"
#include "param.h"

#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"

/* ---------- User-configurable mission params ---------- */
/* heights (meters) */
#define Z1_M    0.50f
#define Z2_M    1.20f
#define TARGET_HEIGHT_M  Z1_M  // Takeoff target (match interrupt_velocity.c pattern)

/* waypoints (world frame, meters) */
#define P1_X_M  2.00f
#define P1_Y_M  0.00f
#define P2_X_M  2.00f
#define P2_Y_M  -2.20f
#define P3_X_M  5.00f
#define P3_Y_M  -2.20f

/* speeds (m/s) */
#define V1_MPS  0.5f
#define V2_MPS  1.0f
#define TAKEOFF_VEL_MPS  0.30f

/* timing */
#define LOOP_HZ            50U
#define FEED_PERIOD_MS     (1000U/LOOP_HZ)
#define HOVER_BETWEEN_MS   500U     // Between legs
#define HOVER_TIME_MS      2000U    // Post-takeoff settle (same as interrupt app)
#define LAND_HOLD_MS       1500U    // Landing hold (same style as interrupt app)
#define ARM_TIMEOUT_MS     2000U

/* arrival thresholds */
#define ARRIVE_DIST_M      0.10f
#define ARRIVE_Z_M         0.05f

/* trigger */
#define AUX_ACTIVE_THRESH  1400

/* fallback */
#ifndef PARAM_VARID_IS_VALID
#define PARAM_VARID_IS_VALID(id) ((id) != (paramVarId_t)0xFFFF)
#endif

/* ---------- Log IDs and flags ---------- */
static logVarId_t idAux3 = 0xFFFFu;
static logVarId_t idX = 0xFFFFu;
static logVarId_t idY = 0xFFFFu;
static logVarId_t idZ = 0xFFFFu;

static bool isActive = false;
static bool sequenceDoneUntilReset = false;

/* ---------- Helpers ---------- */

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

static void holdZ_ms(float zAbs, uint32_t ms)
{
  feedHover_ms(0.0f, 0.0f, zAbs, ms);
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

/* Smooth vertical climb (same as interrupt_velocity.c) */
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

/* Match interrupt_velocity.c: force Kalman + reset */
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
    paramSetInt(idReset, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    paramSetInt(idReset, 0);
  } else {
    DEBUG_PRINT("Param not found: kalman.resetEstimation\n");
  }
}

/* move in XY plane toward (tx,ty) at speed 'speed' (world frame). */
static void moveToXYAtSpeed(float tx, float ty, float speed)
{
  if (!logVarIdIsValid(idX)) idX = logGetVarId("stateEstimate", "x");
  if (!logVarIdIsValid(idY)) idY = logGetVarId("stateEstimate", "y");
  if (!logVarIdIsValid(idZ)) idZ = logGetVarId("stateEstimate", "z");

  float x = logGetFloat(idX), y = logGetFloat(idY);
  while (!isfinite(x) || !isfinite(y)) {
    vTaskDelay(pdMS_TO_TICKS(10));
    x = logGetFloat(idX); y = logGetFloat(idY);
  }

  setpoint_t sp;
  const TickType_t step = pdMS_TO_TICKS(FEED_PERIOD_MS);

  for (;;) {
    x = logGetFloat(idX);
    y = logGetFloat(idY);
    float z = isfinite(logGetFloat(idZ)) ? logGetFloat(idZ) : Z1_M;

    if (!isfinite(x) || !isfinite(y)) { vTaskDelay(step); continue; }

    const float dx = tx - x;
    const float dy = ty - y;
    const float dist = sqrtf(dx*dx + dy*dy);
    if (dist <= ARRIVE_DIST_M) break;

    float vx = 0.0f, vy = 0.0f;
    if (dist > 1e-6f) {
      vx = (dx / dist) * speed;
      vy = (dy / dist) * speed;
    }

    setHoverSetpoint(&sp, vx, vy, z, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(step);
  }

  // stop cleanly
  for (int i = 0; i < 6; ++i) {
    setHoverSetpoint(&sp, 0.0f, 0.0f, logGetFloat(idZ), 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* change altitude to targetZ (meters). */
static void changeAltitudeTo(float targetZ, float climbRate)
{
  if (!logVarIdIsValid(idZ)) idZ = logGetVarId("stateEstimate", "z");
  setpoint_t sp;
  const TickType_t step = pdMS_TO_TICKS(FEED_PERIOD_MS);

  for (;;) {
    float z = logGetFloat(idZ);
    if (!isfinite(z)) { vTaskDelay(step); continue; }
    const float dz = targetZ - z;
    if (fabsf(dz) <= ARRIVE_Z_M) break;

    float stepZ = climbRate * (FEED_PERIOD_MS / 1000.0f);
    if (fabsf(stepZ) < 0.01f) stepZ = 0.01f;
    float zCmd = (dz > 0.0f) ? fminf(z + stepZ, targetZ) : fmaxf(z - stepZ, targetZ);

    setHoverSetpoint(&sp, 0.0f, 0.0f, zCmd, 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(step);
  }

  holdZ_ms(targetZ, 200);
}

/* ---------- Main sequence ---------- */
static void runSequence(void)
{
  DEBUG_PRINT("Waypoint sequence start\n");

  // Exactly like interrupt_velocity.c
  forceKalmanAndReset();

  supervisorRequestArming(true);
  if (!waitForArmed(ARM_TIMEOUT_MS)) {
    DEBUG_PRINT("WARN: did not arm in time\n");
  }

  // Warmup neutral feed
  holdZ_ms(0.0f, 800U);

  // Takeoff to TARGET_HEIGHT_M (which equals Z1_M)
  rampedTakeoff(TARGET_HEIGHT_M, TAKEOFF_VEL_MPS);

  // Settle (same as interrupt app)
  holdZ_ms(TARGET_HEIGHT_M, 1500U);
  holdZ_ms(TARGET_HEIGHT_M, HOVER_TIME_MS);

  // Move to p1 at v1
  moveToXYAtSpeed(P1_X_M, P1_Y_M, V1_MPS);
  holdZ_ms(Z1_M, HOVER_BETWEEN_MS);

  // Move to p2 at v1
  moveToXYAtSpeed(P2_X_M, P2_Y_M, V1_MPS);
  holdZ_ms(Z1_M, HOVER_BETWEEN_MS);

  // Change altitude at p2 to Z2
  changeAltitudeTo(Z2_M, 0.5f); // climb/descent rate ~0.5 m/s
  holdZ_ms(Z2_M, HOVER_BETWEEN_MS);

  // Move to p3 at v2
  moveToXYAtSpeed(P3_X_M, P3_Y_M, V2_MPS);
  holdZ_ms(Z2_M, HOVER_BETWEEN_MS);

  // Land & disarm (same pattern as interrupt app)
  holdZ_ms(0.0f, LAND_HOLD_MS);
  holdZ_ms(0.0f, 200U);
  supervisorRequestArming(false);

  DEBUG_PRINT("Waypoint sequence done\n");
}

/* ---------- App main with trigger ---------- */
void appMain(void)
{
  DEBUG_PRINT("Waiting for activation ...\n");

  TickType_t lastPrint = 0;

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(50));   // 20 Hz poll

    if (!logVarIdIsValid(idAux3)) {
      idAux3 = logGetVarId("cppm", "aux3");
      continue;
    }

    const int16_t aux3 = logGetInt(idAux3);
    const bool nowActive = (aux3 > 0) && (aux3 < AUX_ACTIVE_THRESH);

    if (nowActive && !isActive) {
      DEBUG_PRINT("Activated\n");
      if (!sequenceDoneUntilReset) {
        runSequence();
        sequenceDoneUntilReset = true;
      }
      lastPrint = xTaskGetTickCount();
      isActive = true;
      continue;
    }

    if (!nowActive && isActive) {
      DEBUG_PRINT("Deactivated (re-arm)\n");
      isActive = false;
      sequenceDoneUntilReset = false;
      continue;
    }

    if (isActive && (xTaskGetTickCount() - lastPrint) >= pdMS_TO_TICKS(2000)) {
      DEBUG_PRINT("Still active...\n");
      lastPrint = xTaskGetTickCount();
    }
  }
}
