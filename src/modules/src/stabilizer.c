/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#define DEBUG_MODULE "STAB"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"
#include "platform.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "crtp_localization_service.h"
#include "controller.h"
#include "power_distribution.h"
#include "collision_avoidance.h"
#include "health.h"
#include "supervisor.h"

#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"
#include "rateSupervisor.h"
#include "peer_localization.h"
#include "math3d.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

static uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static motors_thrust_uncapped_t motorThrustUncapped;
static motors_thrust_uncapped_t motorThrustBatCompUncapped;
static motors_thrust_pwm_t motorPwm;

// For scratch storage - never logged or passed to other subsystems.
static setpoint_t tempSetpoint;

static StateEstimatorType estimatorType;
static ControllerType controllerType;

static STATS_CNT_RATE_DEFINE(stabilizerRate, 500);
static rateSupervisor_t rateSupervisorContext;
static bool rateWarningDisplayed = false;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
  
  // payload position - mm
  int16_t px;
  int16_t py;
  int16_t pz;

  // payload velocity - mm / sec
  int16_t pvx;
  int16_t pvy;
  int16_t pvz;
} stateCompressed;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed;

// for payloads
static float payload_alpha_v = 0.9; // between 0...1; 1: no filter
static float payload_alpha_w = 0.9; // between 0...1; 1: no filter
static point_t payload_pos_last;         // m   (world frame)
static quaternion_t payload_quat_last;
static velocity_t payload_vel_last;      // m/s (world frame)
static Axis3f payload_omega_last;

STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void* param);

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

static void compressState()
{
  stateCompressed.x = state.position.x * 1000.0f;
  stateCompressed.y = state.position.y * 1000.0f;
  stateCompressed.z = state.position.z * 1000.0f;

  stateCompressed.vx = state.velocity.x * 1000.0f;
  stateCompressed.vy = state.velocity.y * 1000.0f;
  stateCompressed.vz = state.velocity.z * 1000.0f;

  stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
  stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
  stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;

  float const q[4] = {
    state.attitudeQuaternion.x,
    state.attitudeQuaternion.y,
    state.attitudeQuaternion.z,
    state.attitudeQuaternion.w};
  stateCompressed.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;

  // payload
  stateCompressed.px = state.payload_pos.x * 1000.0f;
  stateCompressed.py = state.payload_pos.y * 1000.0f;
  stateCompressed.pz = state.payload_pos.z * 1000.0f;

  stateCompressed.pvx = state.payload_vel.x * 1000.0f;
  stateCompressed.pvy = state.payload_vel.y * 1000.0f;
  stateCompressed.pvz = state.payload_vel.z * 1000.0f;
}

static void compressSetpoint()
{
  setpointCompressed.x = setpoint.position.x * 1000.0f;
  setpointCompressed.y = setpoint.position.y * 1000.0f;
  setpointCompressed.z = setpoint.position.z * 1000.0f;

  setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
  setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
  setpointCompressed.vz = setpoint.velocity.z * 1000.0f;

  setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
  setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
  setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();
  motorsInit(platformConfigGetMotorMapping());
  collisionAvoidanceInit();
  estimatorType = stateEstimatorGetType();
  controllerType = controllerGetType();

  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();
  pass &= motorsTest();
  pass &= collisionAvoidanceTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

static void batteryCompensation(const motors_thrust_uncapped_t* motorThrustUncapped, motors_thrust_uncapped_t* motorThrustBatCompUncapped)
{
  float supplyVoltage = pmGetBatteryVoltage();

  for (int motor = 0; motor < STABILIZER_NR_OF_MOTORS; motor++)
  {
    motorThrustBatCompUncapped->list[motor] = motorsCompensateBatteryVoltage(motor, motorThrustUncapped->list[motor], supplyVoltage);
  }
}

static void setMotorRatios(const motors_thrust_pwm_t* motorPwm)
{
  motorsSetRatio(MOTOR_M1, motorPwm->motors.m1);
  motorsSetRatio(MOTOR_M2, motorPwm->motors.m2);
  motorsSetRatio(MOTOR_M3, motorPwm->motors.m3);
  motorsSetRatio(MOTOR_M4, motorPwm->motors.m4);
}

/* The stabilizer loop runs at 1kHz. It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */
static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  DEBUG_PRINT("Wait for sensor calibration...\n");

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

  payload_pos_last.timestamp = 0;
   DEBUG_PRINT("Ready to fly.\n");

  while(1) {
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();

    // update sensorData struct (for logging variables)
    sensorsAcquire(&sensorData, tick);

    if (healthShallWeRunTest()) {
      healthRunTests(&sensorData);
    } else {
      // allow to update estimator dynamically
      if (stateEstimatorGetType() != estimatorType) {
        stateEstimatorSwitchTo(estimatorType);
        estimatorType = stateEstimatorGetType();
      }
      // allow to update controller dynamically
      if (controllerGetType() != controllerType) {
        control.controlMode = controlModeLegacy;
        controllerInit(controllerType);
        controllerType = controllerGetType();

        // Make sure we use the correct setpoint (for UAV or payload)
        crtpCommanderHighLevelTellState(&state);
        if (!crtpCommanderHighLevelIsStopped()) {
          // Disable forces the go to command to plan from the current state, rather then current setpoint
          crtpCommanderHighLevelDisable();
          crtpCommanderHighLevelGoTo(0, 0, 0, 0, 1.0, true);
        }
      }

      stateEstimator(&state, tick);

      // add the payload and neighbor states here
      uint8_t num_neighbors = 0;
      for (int i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {

        peerLocalizationOtherPosition_t const *other = peerLocalizationGetPositionByIdx(i);

        if (other == NULL || other->id == 0) {
          continue;
        }

        if (other->id == 255) {
          // handle the payload

          // if we got a new state
          if (payload_pos_last.timestamp < other->timestamp) {
            struct vec vel_filtered = vzero();
            struct vec omega_filtered = vzero();
            // in the beginning, estimate the velocity to be zero, otherwise use
            // numeric estimation with filter
            if (payload_pos_last.timestamp != 0) {
              // estimate the velocity numerically
              const float dt = (other->timestamp - payload_pos_last.timestamp) / 1000.0f; //s
              struct vec pos = mkvec(other->pos.x, other->pos.y, other->pos.z);
              struct vec last_pos = mkvec(payload_pos_last.x, payload_pos_last.y, payload_pos_last.z);
              struct vec vel = vdiv(vsub(pos, last_pos), dt);

              // apply a simple complementary filter
              struct vec vel_old = mkvec(payload_vel_last.x, payload_vel_last.y, payload_vel_last.z);
              vel_filtered = vadd(vscl(1.0f - payload_alpha_v, vel_old), vscl(payload_alpha_v, vel));

              // estimate omega numerically
              struct quat q = mkquat(other->orientation.x, other->orientation.y, other->orientation.z, other->orientation.w);
              struct quat last_q = mkquat(payload_quat_last.x, payload_quat_last.y, payload_quat_last.z, payload_quat_last.w);
              struct vec omega = quat2omega(last_q, q, dt);
              struct vec omega_old = mkvec(payload_omega_last.x, payload_omega_last.y, payload_omega_last.z);
              omega_filtered = vadd(vscl(1.0f - payload_alpha_w, omega_old), vscl(payload_alpha_w, omega));
            }
            // update the position
            state.payload_pos.x = other->pos.x;
            state.payload_pos.y = other->pos.y;
            state.payload_pos.z = other->pos.z;
            state.payload_pos.timestamp = other->timestamp;

            // update the orientation
            state.payload_quat.x = other->orientation.x;
            state.payload_quat.y = other->orientation.y;
            state.payload_quat.z = other->orientation.z;
            state.payload_quat.w = other->orientation.w;
            state.payload_quat.timestamp = other->timestamp;

            // update the velocity
            state.payload_vel.x = vel_filtered.x;
            state.payload_vel.y = vel_filtered.y;
            state.payload_vel.z = vel_filtered.z;
            state.payload_vel.timestamp = other->timestamp;

            // update the angular velocity
            state.payload_omega.x = omega_filtered.x;
            state.payload_omega.y = omega_filtered.y;
            state.payload_omega.z = omega_filtered.z;
            
            // update state
            payload_pos_last = state.payload_pos;
            payload_quat_last = state.payload_quat;
            payload_vel_last = state.payload_vel;
            payload_omega_last = state.payload_omega;
          } else {
            state.payload_pos = payload_pos_last;
            state.payload_quat = payload_quat_last;
            state.payload_vel = payload_vel_last;
            state.payload_omega = payload_omega_last;
          }

        } else if (num_neighbors < MAX_NEIGHBOR_UAVS) {
          // handle regular team members
          state.neighbors[num_neighbors].id = other->id;
          state.neighbors[num_neighbors].pos.x = other->pos.x;
          state.neighbors[num_neighbors].pos.y = other->pos.y;
          state.neighbors[num_neighbors].pos.z = other->pos.z;
          ++num_neighbors;
        }
      }
      state.num_neighbors = num_neighbors;

      compressState();

      if (crtpCommanderHighLevelGetSetpoint(&tempSetpoint, &state, tick)) {
        commanderSetSetpoint(&tempSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
      }

      commanderGetSetpoint(&setpoint, &state);
      compressSetpoint();

      collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, tick);

      controller(&control, &setpoint, &sensorData, &state, tick);

      checkEmergencyStopTimeout();

      //
      // The supervisor module keeps track of Crazyflie state such as if
      // we are ok to fly, or if the Crazyflie is in flight.
      //
      supervisorUpdate(&sensorData);

      if (emergencyStop || (systemIsArmed() == false)) {
        motorsStop();
      } else {
        powerDistribution(&control, &motorThrustUncapped);
        batteryCompensation(&motorThrustUncapped, &motorThrustBatCompUncapped);
        powerDistributionCap(&motorThrustBatCompUncapped, &motorPwm);
        setMotorRatios(&motorPwm);
      }

#ifdef CONFIG_DECK_USD
      // Log data to uSD card if configured
      if (usddeckLoggingEnabled()
          && usddeckLoggingMode() == usddeckLoggingMode_SynchronousStabilizer
          && RATE_DO_EXECUTE(usddeckFrequency(), tick)) {
        usddeckTriggerLogging();
      }
#endif
      calcSensorToOutputLatency(&sensorData);
      tick++;
      STATS_CNT_RATE_EVENT(&stabilizerRate);

      if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount())) {
        if (!rateWarningDisplayed) {
          DEBUG_PRINT("WARNING: stabilizer loop rate is off (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
          rateWarningDisplayed = true;
        }
      }
    }
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
    motorsBurstDshot();
#endif
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/**
 * Parameters to set the estimator and controller type
 * for the stabilizer module, or to do an emergency stop
 */
PARAM_GROUP_START(stabilizer)
/**
 * @brief Estimator type Any(0), complementary(1), extended kalman(2), **unscented kalman(3)  (Default: 0)
 * 
 * ** Experimental, needs to be enabled in kbuild
 */
PARAM_ADD_CORE(PARAM_UINT8, estimator, &estimatorType)
/**
 * @brief Controller type Any(0), PID(1), Mellinger(2), INDI(3), Brescianini(4) (Default: 0)
 */
PARAM_ADD_CORE(PARAM_UINT8, controller, &controllerType)
/**
 * @brief If set to nonzero will turn off power
 */
PARAM_ADD_CORE(PARAM_UINT8, stop, &emergencyStop)


PARAM_ADD_CORE(PARAM_FLOAT, pAlphaV, &payload_alpha_v)

PARAM_ADD_CORE(PARAM_FLOAT, pAlphaW, &payload_alpha_w)

PARAM_GROUP_STOP(stabilizer)


/**
 * Log group for the current controller target
 *
 * Note: all members may not be updated depending on how the system is used
 */
LOG_GROUP_START(ctrltarget)

/**
 * @brief Desired position X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &setpoint.position.x)

/**
 * @brief Desired position Y [m]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &setpoint.position.y)

/**
 * @brief Desired position X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &setpoint.position.z)

/**
 * @brief Desired velocity X [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &setpoint.velocity.x)

/**
 * @brief Desired velocity Y [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vy, &setpoint.velocity.y)

/**
 * @brief Desired velocity Z [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vz, &setpoint.velocity.z)

/**
 * @brief Desired acceleration X [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &setpoint.acceleration.x)

/**
 * @brief Desired acceleration Y [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ay, &setpoint.acceleration.y)

/**
 * @brief Desired acceleration Z [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, az, &setpoint.acceleration.z)

/**
 * @brief Desired attitude, roll [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &setpoint.attitude.roll)

/**
 * @brief Desired attitude, pitch [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, pitch, &setpoint.attitude.pitch)

/**
 * @brief Desired attitude rate, yaw rate [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

/**
 * Log group for the current controller target, compressed format.
 * This flavour of the controller target logs are defined with types
 * that use less space and makes it possible to add more logs to a
 * log configuration.
 *
 * Note: all members may not be updated depending on how the system is used
 */

LOG_GROUP_START(ctrltargetZ)
/**
 * @brief Desired position X [mm]
 */
LOG_ADD(LOG_INT16, x, &setpointCompressed.x)

/**
 * @brief Desired position Y [mm]
 */
LOG_ADD(LOG_INT16, y, &setpointCompressed.y)

/**
 * @brief Desired position Z [mm]
 */
LOG_ADD(LOG_INT16, z, &setpointCompressed.z)

/**
 * @brief Desired velocity X [mm/s]
 */
LOG_ADD(LOG_INT16, vx, &setpointCompressed.vx)

/**
 * @brief Desired velocity Y [mm/s]
 */
LOG_ADD(LOG_INT16, vy, &setpointCompressed.vy)

/**
 * @brief Desired velocity Z [mm/s]
 */
LOG_ADD(LOG_INT16, vz, &setpointCompressed.vz)

/**
 * @brief Desired acceleration X [mm/s^2]
 */
LOG_ADD(LOG_INT16, ax, &setpointCompressed.ax)

/**
 * @brief Desired acceleration Y [mm/s^2]
 */
LOG_ADD(LOG_INT16, ay, &setpointCompressed.ay)

/**
 * @brief Desired acceleration Z [mm/s^2]
 */
LOG_ADD(LOG_INT16, az, &setpointCompressed.az)
LOG_GROUP_STOP(ctrltargetZ)

/**
 * Logs to set the estimator and controller type
 * for the stabilizer module
 */
LOG_GROUP_START(stabilizer)
/**
 * @brief Estimated roll
 *   Note: Same as stateEstimate.roll
 */
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
/**
 * @brief Estimated pitch
 *   Note: Same as stateEstimate.pitch
 */
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
/**
 * @brief Estimated yaw
 *   Note: same as stateEstimate.yaw
 */
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
/**
 * @brief Current thrust
 */
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)
/**
 * @brief Rate of stabilizer loop
 */
STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
/**
 * @brief Latency from sampling of sensor to motor output
 *    Note: Used for debugging but could also be used as a system test
 */
LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
LOG_GROUP_STOP(stabilizer)

/**
 * Log group for accelerometer sensor measurement, based on body frame.
 * Compensated for a miss-alignment by gravity at startup.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what accelerometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(acc)

/**
 * @brief Acceleration in X [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.acc.x)

/**
 * @brief Acceleration in Y [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.acc.y)

/**
 * @brief Acceleration in Z [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

/**
 * Log group for the barometer.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what barometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(baro)

/**
 * @brief Altitude above Sea Level [m]
 */
LOG_ADD_CORE(LOG_FLOAT, asl, &sensorData.baro.asl)

/**
 * @brief Temperature [degrees Celsius]
 */
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)

/**
 * @brief Air preassure [mbar]
 */
LOG_ADD_CORE(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

/**
 * Log group for gyroscopes.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what gyroscope sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(gyro)

/**
 * @brief Angular velocity (rotation) around the X-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.gyro.x)

/**
 * @brief Angular velocity (rotation) around the Y-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.gyro.y)

/**
 * @brief Angular velocity (rotation) around the Z-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

/**
 * Log group for magnetometer.
 *
 * Currently only present on Crazyflie 2.0
 */
LOG_GROUP_START(mag)
/**
 * @brief Magnetometer X axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.mag.x)
/**
 * @brief Magnetometer Y axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.mag.y)
/**
 * @brief Magnetometer Z axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

/**
 * Log group for the state estimator, the currently estimated state of the platform.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
LOG_GROUP_START(stateEstimate)

/**
 * @brief The estimated position of the platform in the global reference frame, X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &state.position.x)

/**
 * @brief The estimated position of the platform in the global reference frame, Y [m]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &state.position.y)

/**
 * @brief The estimated position of the platform in the global reference frame, Z [m]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &state.position.z)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, X [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &state.velocity.x)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Y [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vy, &state.velocity.y)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Z [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vz, &state.velocity.z)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, X [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &state.acc.x)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, Y [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, ay, &state.acc.y)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, without considering gravity, Z [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, az, &state.acc.z)

/**
 * @brief Attitude, roll angle [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &state.attitude.roll)

/**
 * @brief Attitude, pitch angle (legacy CF2 body coordinate system, where pitch is inverted) [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, pitch, &state.attitude.pitch)

/**
 * @brief Attitude, yaw angle [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, yaw, &state.attitude.yaw)

/**
 * @brief Attitude as a quaternion, x
 */
LOG_ADD_CORE(LOG_FLOAT, qx, &state.attitudeQuaternion.x)

/**
 * @brief Attitude as a quaternion, y
 */
LOG_ADD_CORE(LOG_FLOAT, qy, &state.attitudeQuaternion.y)

/**
 * @brief Attitude as a quaternion, z
 */
LOG_ADD_CORE(LOG_FLOAT, qz, &state.attitudeQuaternion.z)

/**
 * @brief Attitude as a quaternion, w
 */
LOG_ADD_CORE(LOG_FLOAT, qw, &state.attitudeQuaternion.w)


/**
 * @brief The position of the payload in the global reference frame, X [m]
 */
LOG_ADD(LOG_FLOAT, px, &state.payload_pos.x)

/**
 * @brief The position of the payload in the global reference frame, Y [m]
 */
LOG_ADD(LOG_FLOAT, py, &state.payload_pos.y)

/**
 * @brief The position of the payload in the global reference frame, Z [m]
 */
LOG_ADD(LOG_FLOAT, pz, &state.payload_pos.z)

/**
 * @brief The orientation of the payload in the global reference frame
 */
LOG_ADD(LOG_FLOAT, pqx, &state.payload_quat.x)

/**
 * @brief The orientation of the payload in the global reference frame
 */
LOG_ADD(LOG_FLOAT, pqy, &state.payload_quat.y)

/**
 * @brief The orientation of the payload in the global reference frame
 */
LOG_ADD(LOG_FLOAT, pqz, &state.payload_quat.z)

/**
 * @brief The orientation of the payload in the global reference frame
 */
LOG_ADD(LOG_FLOAT, pqw, &state.payload_quat.w)

/**
 * @brief The velocity of the payload in the global reference frame, X [m/s]
 */
LOG_ADD(LOG_FLOAT, pvx, &state.payload_vel.x)

/**
 * @brief The velocity of the payload in the global reference frame, Y [m/s]
 */
LOG_ADD(LOG_FLOAT, pvy, &state.payload_vel.y)

/**
 * @brief The velocity of the payload in the global reference frame, Z [m/s]
 */
LOG_ADD(LOG_FLOAT, pvz, &state.payload_vel.z)

/**
 * @brief Angular velocity of the payload, X [rad/s]
 */
LOG_ADD(LOG_FLOAT, pwx, &state.payload_omega.x)

/**
 * @brief Angular velocity of the payload, Y [rad/s]
 */
LOG_ADD(LOG_FLOAT, pwy, &state.payload_omega.y)

/**
 * @brief Angular velocity of the payload, Z [rad/s]
 */
LOG_ADD(LOG_FLOAT, pwz, &state.payload_omega.z)

LOG_GROUP_STOP(stateEstimate)

/**
 * Log group for the state estimator, compressed format. This flavour of the
 * estimator logs are defined with types that use less space and makes it possible to
 * add more logs to a log configuration.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
LOG_GROUP_START(stateEstimateZ)

/**
 * @brief The position of the Crazyflie in the global reference frame, X [mm]
 */
LOG_ADD(LOG_INT16, x, &stateCompressed.x)

/**
 * @brief The position of the Crazyflie in the global reference frame, Y [mm]
 */
LOG_ADD(LOG_INT16, y, &stateCompressed.y)

/**
 * @brief The position of the Crazyflie in the global reference frame, Z [mm]
 */
LOG_ADD(LOG_INT16, z, &stateCompressed.z)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, X [mm/s]
 */
LOG_ADD(LOG_INT16, vx, &stateCompressed.vx)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Y [mm/s]
 */
LOG_ADD(LOG_INT16, vy, &stateCompressed.vy)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Z [mm/s]
 */
LOG_ADD(LOG_INT16, vz, &stateCompressed.vz)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, X [mm/s]
 */
LOG_ADD(LOG_INT16, ax, &stateCompressed.ax)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, Y [mm/s]
 */
LOG_ADD(LOG_INT16, ay, &stateCompressed.ay)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, including gravity, Z [mm/s]
 */
LOG_ADD(LOG_INT16, az, &stateCompressed.az)

/**
 * @brief Attitude as a compressed quaternion, see see quatcompress.h for details
 */
LOG_ADD(LOG_UINT32, quat, &stateCompressed.quat)

/**
 * @brief Roll rate (angular velocity) [milliradians / sec]
 */
LOG_ADD(LOG_INT16, rateRoll, &stateCompressed.rateRoll)

/**
 * @brief Pitch rate (angular velocity) [milliradians / sec]
 */
LOG_ADD(LOG_INT16, ratePitch, &stateCompressed.ratePitch)

/**
 * @brief Yaw rate (angular velocity) [milliradians / sec]
 */
LOG_ADD(LOG_INT16, rateYaw, &stateCompressed.rateYaw)


/**
 * @brief The position of the payload in the global reference frame, X [mm]
 */
LOG_ADD(LOG_INT16, px, &stateCompressed.px)

/**
 * @brief The position of the payload in the global reference frame, Y [mm]
 */
LOG_ADD(LOG_INT16, py, &stateCompressed.py)

/**
 * @brief The position of the payload in the global reference frame, Z [mm]
 */
LOG_ADD(LOG_INT16, pz, &stateCompressed.pz)

/**
 * @brief The velocity of the payload in the global reference frame, X [mm/s]
 */
LOG_ADD(LOG_INT16, pvx, &stateCompressed.pvx)

/**
 * @brief The velocity of the payload in the global reference frame, Y [mm/s]
 */
LOG_ADD(LOG_INT16, pvy, &stateCompressed.pvy)

/**
 * @brief The velocity of the payload in the global reference frame, Z [mm/s]
 */
LOG_ADD(LOG_INT16, pvz, &stateCompressed.pvz)


LOG_GROUP_STOP(stateEstimateZ)


LOG_GROUP_START(motor)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m1req, &motorThrustBatCompUncapped.motors.m1)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m2req, &motorThrustBatCompUncapped.motors.m2)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m3req, &motorThrustBatCompUncapped.motors.m3)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m4req, &motorThrustBatCompUncapped.motors.m4)
LOG_GROUP_STOP(motor)
