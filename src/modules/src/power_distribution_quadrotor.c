/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "platform_defaults.h"
#include "pm.h"
#include "motors.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
static float armLength = ARM_LENGTH; // m
static float thrustToTorque = 0.005964552f;

// thrust = a * pwm^2 + b * pwm
//    where PWM is normalized (range 0...1)
//          thrust is in Newtons (per rotor)
static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;

// voltage, thrust -> pwm
static float d00 = 0.5543364748044269;
static float d10 = 0.11442589787133063;
static float d01 = -0.5067031467944692;
static float d20 = -0.002283966554392003;
static float d11 = -0.03255320005438393;

// voltage, pwm -> max_thrust
static float e00 = -10.291152501242268;
static float e10 = 3.926415845326646;
static float e01 = 26.077196474667165;

// // computes maximum thrust in grams given the current battery state
// static float estimateMaxThrust(float batteryVoltage)
// {
//   // normalized voltage
//   float v = pmGetBatteryVoltage() / 4.2f;
//   // normalized (average) pwm
//   uint32_t pwm_sum = 0;
//   for (int i = 0; i < STABILIZER_NR_OF_MOTORS; ++i) {
//     pwm_sum += motorsGetRatio(i);
//   }
//   float pwm = pwm_sum / 4.0f / UINT16_MAX;

//   float maxThrust = clamp(e00 + e10 * pwm + e01 * v, 8, 50);

//   return maxThrust;
// }

// set thrust for motor (in grams)
static uint16_t thrustToPWM(float batteryVoltage, float thrustGram)
{
  if (thrustGram > 0) {
    // normalized voltage
    float v = batteryVoltage / 4.2f;
    // normalized pwm:
    float pwm = d00 + d10 * thrustGram + d01 * v + d20 * thrustGram * thrustGram + d11 * thrustGram * v;

    return pwm * UINT16_MAX;
  }

  return 0;
}

int powerDistributionMotorType(uint32_t id)
{
  return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  return 0;
}

void powerDistributionInit(void)
{
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

static uint16_t capMinThrust(float thrust, uint32_t minThrust) {
  if (thrust < minThrust) {
    return minThrust;
  }

  return thrust;
}

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;

  motorThrustUncapped->motors.m1 = control->thrust - r + p + control->yaw;
  motorThrustUncapped->motors.m2 = control->thrust - r - p - control->yaw;
  motorThrustUncapped->motors.m3 = control->thrust + r - p + control->yaw;
  motorThrustUncapped->motors.m4 = control->thrust + r + p - control->yaw;
}

static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  static float motorForces[STABILIZER_NR_OF_MOTORS];

  const float arm = 0.707106781f * armLength;
  const float rollPart = 0.25f / arm * control->torqueX;
  const float pitchPart = 0.25f / arm * control->torqueY;
  const float thrustPart = 0.25f * control->thrustSi; // N (per rotor)
  const float yawPart = 0.25f * control->torqueZ / thrustToTorque;

  motorForces[0] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[1] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[2] = thrustPart + rollPart + pitchPart - yawPart;
  motorForces[3] = thrustPart + rollPart - pitchPart + yawPart;

  float batteryVoltage = pmGetBatteryVoltage();
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++) {
    float motorForce = motorForces[motorIndex];
    if (motorForce < 0.0f) {
      motorForce = 0.0f;
    }

    // float motor_pwm = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * motorForce)) / (2.0f * pwmToThrustA);
    // motorThrustUncapped->list[motorIndex] = motor_pwm * UINT16_MAX;

    float motorForceInGrams = motorForce * 1000.0f / 9.81f;
    motorThrustUncapped->list[motorIndex] = thrustToPWM(batteryVoltage, motorForceInGrams);
  }
}

static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  // Not implemented yet
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
      powerDistributionLegacy(control, motorThrustUncapped);
      break;
    case controlModeForceTorque:
      powerDistributionForceTorque(control, motorThrustUncapped);
      break;
    case controlModeForce:
      powerDistributionForce(control, motorThrustUncapped);
      break;
    default:
      // Nothing here
      break;
  }
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  const int32_t maxAllowedThrust = UINT16_MAX;
  bool isCapped = false;

  // Find highest thrust
  int32_t highestThrustFound = 0;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t thrust = motorThrustBatCompUncapped->list[motorIndex];
    if (thrust > highestThrustFound)
    {
      highestThrustFound = thrust;
    }
  }

  int32_t reduction = 0;
  if (highestThrustFound > maxAllowedThrust)
  {
    reduction = highestThrustFound - maxAllowedThrust;
    isCapped = true;
  }

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    int32_t thrustCappedUpper = motorThrustBatCompUncapped->list[motorIndex] - reduction;
    motorPwm->list[motorIndex] = capMinThrust(thrustCappedUpper, idleThrust);
  }

  return isCapped;
}

uint32_t powerDistributionGetIdleThrust() {
  return idleThrust;
}

float powerDistributionGetMaxThrust() {
  // max thrust per rotor occurs if normalized PWM is 1
  // pwmToThrustA * pwm * pwm + pwmToThrustB * pwm = pwmToThrustA + pwmToThrustB
  return STABILIZER_NR_OF_MOTORS * (pwmToThrustA + pwmToThrustB);
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

/**
 * System identification parameters for quad rotor
 */
PARAM_GROUP_START(quadSysId)

PARAM_ADD(PARAM_FLOAT, thrustToTorque, &thrustToTorque)
PARAM_ADD(PARAM_FLOAT, pwmToThrustA, &pwmToThrustA)
PARAM_ADD(PARAM_FLOAT, pwmToThrustB, &pwmToThrustB)

/**
 * @brief Length of arms (m)
 *
 * The distance from the center to a motor
 */
PARAM_ADD(PARAM_FLOAT, armLength, &armLength)
PARAM_GROUP_STOP(quadSysId)


/**
 * ToDo: This should move into quadSysId and replace pwmToThrust etc.
 */
PARAM_GROUP_START(pwm)

  PARAM_ADD(PARAM_FLOAT, d00, &d00)
  PARAM_ADD(PARAM_FLOAT, d10, &d10)
  PARAM_ADD(PARAM_FLOAT, d01, &d01)
  PARAM_ADD(PARAM_FLOAT, d20, &d20)
  PARAM_ADD(PARAM_FLOAT, d11, &d11)

  PARAM_ADD(PARAM_FLOAT, e00, &e00)
  PARAM_ADD(PARAM_FLOAT, e10, &e10)
  PARAM_ADD(PARAM_FLOAT, e01, &e01)

PARAM_GROUP_STOP(pwm)