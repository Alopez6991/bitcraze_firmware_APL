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
#define P1_X_M  1.85f
#define P1_Y_M  0.00f
#define P2_X_M  1.85f
#define P2_Y_M  -2.50f
#define P3_X_M  5.00f
#define P3_Y_M  -2.50f

//  test values
// /* waypoints (world frame, meters) */
// #define P1_X_M  1.00f
// #define P1_Y_M  0.00f
// #define P2_X_M  1.00f
// #define P2_Y_M  -1.50f
// #define P3_X_M  2.00f
// #define P3_Y_M  -1.50f

//red gate code

// /* ---------- User-configurable mission params ---------- */
// /* heights (meters) */
// #define Z1_M    0.50f
// #define Z2_M    1.60f
// #define TARGET_HEIGHT_M  Z1_M  // Takeoff target (match interrupt_velocity.c pattern)

// /* waypoints (world frame, meters) */
// #define P1_X_M  1.85f
// #define P1_Y_M  0.00f
// #define P2_X_M  1.85f
// #define P2_Y_M  -3.60f
// #define P3_X_M  5.00f
// #define P3_Y_M  -3.60f

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

/* KILL switch: disarm immediately (no landing) */
#define KILL_SWITCH_GROUP  "cppm"
#define KILL_SWITCH_NAME   "aux0"
#define KILL_ACTIVE_THRESH 1400

/* fallback */
#ifndef PARAM_VARID_IS_VALID
#define PARAM_VARID_IS_VALID(id) ((id) != (paramVarId_t)0xFFFF)
#endif

/* ---------- Log IDs and flags ---------- */
static logVarId_t idAux3 = 0xFFFFu;
static logVarId_t idKill = 0xFFFFu;
static logVarId_t idX = 0xFFFFu;
static logVarId_t idY = 0xFFFFu;
static logVarId_t idZ = 0xFFFFu;

static bool isActive = false;
static bool sequenceDoneUntilReset = false;

/* ---------- Helpers ---------- */
static inline bool readSwitchActive_threshold(logVarId_t id, int thr) {
  if (!logVarIdIsValid(id)) return false;
  const int16_t v = logGetInt(id);
  return (v > 0) && (v < thr);
}
static inline void ensureKillId(void) {
  if (!logVarIdIsValid(idKill)) {
    idKill = logGetVarId(KILL_SWITCH_GROUP, KILL_SWITCH_NAME);
  }
}
static inline bool checkKillAndDisarm(void) {
  ensureKillId();
  if (logVarIdIsValid(idKill) && readSwitchActive_threshold(idKill, KILL_ACTIVE_THRESH)) {
    DEBUG_PRINT("KILL: disarm NOW\n");
    supervisorRequestArming(false);  // immediate disarm
    return true;
  }
  return false;
}
static inline bool isDisarmed(void) {
  return !supervisorIsArmed();
}

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
    if (checkKillAndDisarm() || isDisarmed()) return;
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
    if (checkKillAndDisarm()) return false;
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
    if (checkKillAndDisarm() || isDisarmed()) return;
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
    if (checkKillAndDisarm() || isDisarmed()) return;
    vTaskDelay(pdMS_TO_TICKS(10));
    x = logGetFloat(idX); y = logGetFloat(idY);
  }

  setpoint_t sp;
  const TickType_t step = pdMS_TO_TICKS(FEED_PERIOD_MS);

  for (;;) {
    if (checkKillAndDisarm() || isDisarmed()) return;
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
    if (checkKillAndDisarm() || isDisarmed()) return;
    setHoverSetpoint(&sp, 0.0f, 0.0f, logGetFloat(idZ), 0.0f, true);
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* change altitude to targetZ using vertical velocity mode (faster, tracks climbRate) */
static void changeAltitudeTo(float targetZ, float climbRate)
{
  if (climbRate < 0.10f) climbRate = 0.10f;   // safety floor
  if (climbRate > 1.50f) climbRate = 1.50f;   // keep reasonable

  if (!logVarIdIsValid(idZ)) idZ = logGetVarId("stateEstimate", "z");
  const TickType_t step = pdMS_TO_TICKS(FEED_PERIOD_MS);
  setpoint_t sp;

  for (;;) {
    if (checkKillAndDisarm() || isDisarmed()) return;
    float z = logGetFloat(idZ);
    if (!isfinite(z)) { vTaskDelay(step); continue; }

    const float dz = targetZ - z;
    if (fabsf(dz) <= ARRIVE_Z_M) break;

    const float vz = copysignf(climbRate, dz);

    memset(&sp, 0, sizeof(sp));
    sp.mode.x = modeVelocity;  sp.mode.y = modeVelocity;
    sp.velocity.x = 0.0f;      sp.velocity.y = 0.0f;

    sp.mode.z = modeVelocity;
    sp.velocity.z = vz;

    sp.mode.yaw = modeVelocity;
    sp.attitudeRate.yaw = 0.0f;

    sp.velocity_body = false; // world frame
    commanderSetSetpoint(&sp, 3);
    vTaskDelay(step);
  }

  // Brake vertical velocity and settle at target
  memset(&sp, 0, sizeof(sp));
  sp.mode.x = modeVelocity;  sp.mode.y = modeVelocity;
  sp.velocity.x = 0.0f;      sp.velocity.y = 0.0f;

  sp.mode.z = modeVelocity;
  sp.velocity.z = 0.0f;

  sp.mode.yaw = modeVelocity;
  sp.attitudeRate.yaw = 0.0f;

  sp.velocity_body = false;
  commanderSetSetpoint(&sp, 3);

  holdZ_ms(targetZ, 200);
}

/* ---------- Main sequence ---------- */
static void runSequence(void)
{
  DEBUG_PRINT("Waypoint sequence start\n");
  if (checkKillAndDisarm()) return;

  // Exactly like interrupt_velocity.c
  forceKalmanAndReset();

  supervisorRequestArming(true);
  if (!waitForArmed(ARM_TIMEOUT_MS)) {
    DEBUG_PRINT("WARN: did not arm in time\n");
  }

  // Warmup neutral feed
  holdZ_ms(0.0f, 800U);
  if (checkKillAndDisarm() || isDisarmed()) return;

  // Takeoff to TARGET_HEIGHT_M (which equals Z1_M)
  rampedTakeoff(TARGET_HEIGHT_M, TAKEOFF_VEL_MPS);
  if (checkKillAndDisarm() || isDisarmed()) return;

  // Settle (same as interrupt app)
  holdZ_ms(TARGET_HEIGHT_M, 1500U);
  if (checkKillAndDisarm() || isDisarmed()) return;
  holdZ_ms(TARGET_HEIGHT_M, HOVER_TIME_MS);
  if (checkKillAndDisarm() || isDisarmed()) return;

  // Move to p1 at v1
  moveToXYAtSpeed(P1_X_M, P1_Y_M, V1_MPS);
  holdZ_ms(Z1_M, HOVER_BETWEEN_MS);
  if (checkKillAndDisarm() || isDisarmed()) return;

  // Move to p2 at v1
  moveToXYAtSpeed(P2_X_M, P2_Y_M, V1_MPS);
  holdZ_ms(Z1_M, HOVER_BETWEEN_MS);
  if (checkKillAndDisarm() || isDisarmed()) return;

  // Change altitude at p2 to Z2 (faster climb)
  changeAltitudeTo(Z2_M, 0.5f);
  holdZ_ms(Z2_M, HOVER_BETWEEN_MS);
  if (checkKillAndDisarm() || isDisarmed()) return;

  // Move to p3 at v2
  moveToXYAtSpeed(P3_X_M, P3_Y_M, V2_MPS);
  holdZ_ms(Z2_M, HOVER_BETWEEN_MS);
  if (checkKillAndDisarm() || isDisarmed()) return;

  // Land & disarm (same pattern as interrupt app)
  holdZ_ms(0.0f, LAND_HOLD_MS);
  holdZ_ms(0.0f, 200U);
  supervisorRequestArming(false);

  DEBUG_PRINT("Waypoint sequence done\n");
}

/* ---------- App main with trigger ---------- */
void appMain(void)
{
  DEBUG_PRINT("Waiting for activation ... (START=cppm.aux3<%d, KILL=cppm.aux0<%d)\n",
              AUX_ACTIVE_THRESH, KILL_ACTIVE_THRESH);

  TickType_t lastPrint = 0;

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(50));   // 20 Hz poll

    // Resolve IDs if needed
    if (!logVarIdIsValid(idAux3)) {
      idAux3 = logGetVarId("cppm", "aux3");
    }
    ensureKillId();

    // KILL first (interrupts everything) -> disarm only
    if (checkKillAndDisarm()) {
      isActive = false;
      sequenceDoneUntilReset = false;
      continue;
    }

    if (!logVarIdIsValid(idAux3)) { continue; }

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

    if (isActive && (xTaskGetTickCount() - lastPrint) >= pdMS_TO_TICKS(2000)) {
      DEBUG_PRINT("Switch still active…\n");
      lastPrint = xTaskGetTickCount();
    }
  }
}
