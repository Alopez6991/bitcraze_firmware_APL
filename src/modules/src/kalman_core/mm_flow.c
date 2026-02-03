/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "mm_flow.h"
#include "log.h"
#include "param.h"

#define FLOW_RESOLUTION 0.10f //We do get the measurements in 10x the motion pixels (experimentally measured)

// ============================================================================
// Gyro low-pass filter for oscillating platforms (e.g., flappers)
// ============================================================================
static Axis3f gyroFiltered = {0};
static bool gyroFilterInitialized = false;

// Filter time constant in seconds. Set to 0 to disable filtering.
// For 20-30 Hz oscillations, tau = 0.03-0.04s gives good attenuation.
// f_c = 1/(2*pi*tau), e.g. tau=0.03 -> f_c ~5Hz, attenuates 25Hz by ~80%
static float gyroFilterTauS = 0.0f;

// TODO remove the temporary test variables (used for logging)
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;

void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro, const bool isFlying)
{
  // Inclusion of flow measurements in the EKF done by two scalar updates
  
  // ~~~ Gyro filtering for oscillating platforms ~~~
  // Apply low-pass filter to gyro to match flow sensor integration behavior
  // and attenuate high-frequency oscillations (e.g., 20-30 Hz flapping)
  Axis3f gyroToUse;
  
  if (gyroFilterTauS > 0.0f) {
    if (!gyroFilterInitialized) {
      gyroFiltered = *gyro;
      gyroFilterInitialized = true;
    } else {
      // EMA: alpha = dt / tau (clamped to 1.0)
      float alpha = flow->dt / gyroFilterTauS;
      if (alpha > 1.0f) alpha = 1.0f;
      
      gyroFiltered.x += alpha * (gyro->x - gyroFiltered.x);
      gyroFiltered.y += alpha * (gyro->y - gyroFiltered.y);
      gyroFiltered.z += alpha * (gyro->z - gyroFiltered.z);
    }
    gyroToUse = gyroFiltered;
  } else {
    // No filtering, use raw gyro (default, matches stock firmware)
    gyroToUse = *gyro;
  }

  // ~~~ Camera constants ~~~
  // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
  float Npix = 35.0;                      // [pixels] (same in x and y)
  //float thetapix = DEG_TO_RAD * 4.0f;     // [rad]    (same in x and y)
  float thetapix = 0.71674f;// 2*sin(42/2); 42degree is the agnle of aperture, here we computed the corresponding ground length
  //~~~ Body rates ~~~
  // Use filtered gyro for flow prediction
  float omegax_b = gyroToUse.x * DEG_TO_RAD;
  float omegay_b = gyroToUse.y * DEG_TO_RAD;

  // ~~~ Moves the body velocity into the global coordinate system ~~~
  // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
  //
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  //
  // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
  // _G and _B refer to the global/body coordinate systems.

  // Modification 1
  //dx_g = R[0][0] * S[KC_STATE_PX] + R[0][1] * S[KC_STATE_PY] + R[0][2] * S[KC_STATE_PZ];
  //dy_g = R[1][0] * S[KC_STATE_PX] + R[1][1] * S[KC_STATE_PY] + R[1][2] * S[KC_STATE_PZ];


  float dx_g = this->S[KC_STATE_PX];
  float dy_g = this->S[KC_STATE_PY];
  float z_g = 0.0;
  // Saturate elevation in prediction and correction to avoid singularities
  if ( this->S[KC_STATE_Z] < 0.1f ) {
      z_g = 0.1;
  } else {
      z_g = this->S[KC_STATE_Z];
  }

  // ~~~ X velocity prediction and update ~~~
  // predicts the number of accumulated pixels in the x-direction
  float hx[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix ) * ((dx_g * this->R[2][2] / z_g) - omegay_b);
  measuredNX = flow->dpixelx*FLOW_RESOLUTION;

  // derive measurement equation with respect to dx (and z?)
  hx[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((this->R[2][2] * dx_g) / (-z_g * z_g));
  hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);


  //First update
  if (!isFlying) {
    kalmanCoreScalarUpdate(this, &Hx, (0.0f-predictedNX), 0.0f);
  }


  if (isFlying && (this->S[KC_STATE_Z] > 0.12f)) {
    kalmanCoreScalarUpdate(this, &Hx, (measuredNX-predictedNX), flow->stdDevX*FLOW_RESOLUTION);
  }

  // ~~~ Y velocity prediction and update ~~~
  float hy[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix ) * ((dy_g * this->R[2][2] / z_g) + omegax_b);
  measuredNY = flow->dpixely*FLOW_RESOLUTION;

  // derive measurement equation with respect to dy (and z?)
  hy[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((this->R[2][2] * dy_g) / (-z_g * z_g));
  hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);

  if (!isFlying) {
    kalmanCoreScalarUpdate(this, &Hy, (0.0f-predictedNY), 0.0f);
  }

  if (isFlying && (this->S[KC_STATE_Z] > 0.12f)) {
    kalmanCoreScalarUpdate(this, &Hy, (measuredNY-predictedNY), flow->stdDevY*FLOW_RESOLUTION);
  }
}

/**
 * Predicted and measured values of the X and Y direction of the flowdeck
 */
LOG_GROUP_START(kalman_pred)

/**
 * @brief Flow sensor predicted dx  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowX?
 */
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
/**
 * @brief Flow sensor predicted dy  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowY?
 */
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
/**
 * @brief Flow sensor measured dx  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaX, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
/**
 * @brief Flow sensor measured dy  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaY, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)

/**
 * Parameters for gyro filtering in flow prediction
 */
PARAM_GROUP_START(flowFilter)
/**
 * @brief Gyro low-pass filter time constant in seconds
 * 
 * For oscillating platforms (flappers), set to 0.03-0.04s to attenuate
 * 20-30 Hz body oscillations. This improves flow prediction accuracy
 * by matching the gyro signal to what the flow sensor integrates.
 * f_c = 1/(2*pi*tau), e.g. tau=0.03 -> f_c ~5Hz
 * Set to 0 to disable filtering (default, matches stock firmware).
 */
  PARAM_ADD(PARAM_FLOAT, gyroTauS, &gyroFilterTauS)
PARAM_GROUP_STOP(flowFilter)
