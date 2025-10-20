#include <math.h>
#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#include "math3d.h"
#include "controller_lee_payload.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "usec_time.h"
#include "motors.h"
#include "filter.h"


static controllerLeePayload_t g_self = {
  .mass = 0.0366,
  .mp   = 0.0048,
  // Inertia matrix (diagonal matrix), see
  // System Identification of the Crazyflie 2.0 Nano Quadrocopter
  // BA theses, Julian Foerster, ETHZ
  // https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  // Payload PID
  .Kpos_P = {6.0, 6.0, 6.0}, // Kp in paper
  .Kpos_P_limit = 100,
  .Kpos_D = {4.0, 4.0, 4.0}, // Kv in paper
  .Kpos_D_limit = 100,
  .Kpos_I = {1.0, 1.0, 3.0}, // not in paper
  .Kpos_I_limit = 100,

  // Cable PD 
  .K_q = {8.0, 8.0, 8.0}, // cable direction P
  .K_q_limit = 100,
  .K_w = {7.5, 7.5, 7.5}, // cable angular velocity D
  .K_w_limit = 100,
  .K_q_I = {0.0, 0.0, 0.0}, // cable direction I

  // UAV Position PID Gains
  .Kpos_UAV_P = {0.0, 0.0, 0.0}, // UAV position Kp
  .Kpos_UAV_P_limit = 100,
  .Kpos_UAV_D = {0.0, 0.0, 0.0}, // UAV position Kv
  .Kpos_UAV_D_limit = 100,
  .Kpos_UAV_I = {0.0, 0.0, 0.0}, // UAV position Ki
  .Kpos_UAV_I_limit = 100,

  // UAV Attitude PID
  .KR = {0.0085, 0.0085, 0.0085},
  .Komega = {0.0013, 0.0013, 0.0013},
  .KI = {0.01, 0.01, 0.01},

  .attachement_points[0].l = 0.5,
  .attachement_points[1].l = 0.5,
  .attachement_points[2].l = 0.5,

  // INDI
  .indi = 0,
  .use_flat_output = 0,

};


static bool rpm_deck_available;
static logVarId_t logVarRpm1;
static logVarId_t logVarRpm2;
static logVarId_t logVarRpm3;
static logVarId_t logVarRpm4;

static Butterworth2LowPass filter_acc_rpm[3];
static Butterworth2LowPass filter_acc_imu[3];
static Butterworth2LowPass filter_tau_rpm[3];
static Butterworth2LowPass filter_tau_imu[3];


extern float rpm2pwmA;
extern float rpm2pwmB;
extern float kappa_f[4];

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

void controllerLeePayloadReset(controllerLeePayload_t* self)
{
  self->i_error_pos = vzero();
  self->i_error_q = vzero();
  self->i_error_pos_uav = vzero();
  self->i_error_att_uav = vzero();
}


void controllerLeePayloadInit(controllerLeePayload_t* self)
{
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;

  paramVarId_t idDeckBcRpm = paramGetVarId("deck", "bcRpm");
  logVarRpm1 = logGetVarId("rpm", "m1");
  logVarRpm2 = logGetVarId("rpm", "m2");
  logVarRpm3 = logGetVarId("rpm", "m3");
  logVarRpm4 = logGetVarId("rpm", "m4");

  rpm_deck_available = (paramGetUint(idDeckBcRpm) == 1);

  const float cutoff_acc = 100; // Hz
	for (int8_t i = 0; i < 3; i++) {
		init_butterworth_2_low_pass(&filter_acc_rpm[i], 1 / (2 * M_PI_F * cutoff_acc), 1.0 / ATTITUDE_RATE, 0.0f);
		init_butterworth_2_low_pass(&filter_acc_imu[i], 1 / (2 * M_PI_F * cutoff_acc), 1.0 / ATTITUDE_RATE, 0.0f);
  }
  const float cutoff_tau = 100; // Hz
	for (int8_t i = 0; i < 2; i++) {
		init_butterworth_2_low_pass(&filter_tau_rpm[i], 1 / (2 * M_PI_F * cutoff_tau), 1.0 / ATTITUDE_RATE, 0.0f);
		init_butterworth_2_low_pass(&filter_tau_imu[i], 1 / (2 * M_PI_F * cutoff_tau), 1.0 / ATTITUDE_RATE, 0.0f);
	}
  const float cutoff_z = 10; // Hz
  init_butterworth_2_low_pass(&filter_tau_rpm[2], 1 / (2 * M_PI_F * cutoff_z), 1.0 / ATTITUDE_RATE, 0.0f);
  init_butterworth_2_low_pass(&filter_tau_imu[2], 1 / (2 * M_PI_F * cutoff_z), 1.0 / ATTITUDE_RATE, 0.0f);


  if (rpm_deck_available && (self->indi == 3)) {
    DEBUG_PRINT("Using INDI (both)\n");
  } else if (rpm_deck_available && (self->indi == 1)){
    DEBUG_PRINT("Using INDI (acc)\n");
  } else if (rpm_deck_available && (self->indi == 2)){
    DEBUG_PRINT("Using INDI (gyro)\n");
  } else {
    DEBUG_PRINT("No INDI\n");
  }

  self->timestamp_prev = usecTimestamp();
  self->omega_prev = vzero();

  controllerLeePayloadReset(self);
}

void controllerLeePayload(controllerLeePayload_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }
  float dt = (float)(1.0f/ATTITUDE_RATE);
  float desiredYaw = 0; //rad
  
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = radians(setpoint->attitude.yaw);
  }

  // INDI
  float t1 = 0.0f, t2 = 0.0f, t3 = 0.0f, t4 = 0.0f;
  if (self->indi && rpm_deck_available) {

    uint16_t rpm[4];

    if (self-> indi & 4) {
      // compute force based on PWM measurements

      // pwm_normalized = rpm2pwmA + b * rpm
      // rpm = (pwm_normalized - rpm2pwmA) / rpm2pwmB

      for (int i = 0; i < 4; ++i) {
        float pwm_normalized = motorsGetRatio(i) / 65535.0f;
        rpm[i] = (pwm_normalized - rpm2pwmA) / rpm2pwmB;
      }


    } else {
      // compute force based on RPM measurements
      rpm[0] = logGetUint(logVarRpm1);
      rpm[1] = logGetUint(logVarRpm2);
      rpm[2] = logGetUint(logVarRpm3);
      rpm[3] = logGetUint(logVarRpm4);
    }

    t1 = kappa_f[0] * powf(rpm[0], 2);
    t2 = kappa_f[1] * powf(rpm[1], 2);
    t3 = kappa_f[2] * powf(rpm[2], 2);
    t4 = kappa_f[3] * powf(rpm[3], 2);

    // // DEBUG
    // if (tick % 500 == 0) {
      
    //   DEBUG_PRINT("INDI t %f %f %f %f\n", (double)t1, (double)t2, (double)t3, (double)t4);
    // }
  }

  struct vec xc = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0);
  struct vec yc = mkvec(-sinf(desiredYaw), cosf(desiredYaw), 0);


  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {
    struct vec gravity_comp = mkvec(0, 0, GRAVITY_MAGNITUDE);
    // payload desired trajectory (from setpoint)
    struct vec plPos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec plVel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec plAcc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z);
    self->plJerk_d = mkvec(setpoint->jerk.x, setpoint->jerk.y, setpoint->jerk.z);
    self->plSnap_d = mkvec(setpoint->snap.x, setpoint->snap.y, setpoint->snap.z);
    // self->pldSnap_d = mkvec(setpoint->snapd.x, setpoint->snapd.y, setpoint->snapd.z); 
    self->pldSnap_d = vzero(); // set to zero for now
    // struct vec plddSnap_d = vzero(); // set to zero for now

    // payload position and velocity states
    struct vec plPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
    struct vec plVel = mkvec(state->payload_vel.x, state->payload_vel.y, state->payload_vel.z);
    struct vec plAcc = mkvec(state->payload_acc.x, state->payload_acc.y, state->payload_acc.z);
    
    // UAV position and velocity states
    struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
    struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
        
    // position and velocity errors
    struct vec plpos_e = vclampscl(vsub(plPos_d, plPos), -self->Kpos_P_limit, self->Kpos_P_limit);
    struct vec plvel_e = vclampscl(vsub(plVel_d, plVel), -self->Kpos_D_limit, self->Kpos_D_limit);
    self->i_error_pos = vadd(self->i_error_pos, vscl(dt, plpos_e));

  
    struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
    struct mat33 R = quat2rotmat(q);
    struct vec z  = vbasis(2);
    struct vec R_z = mvmul(R, z);
    // cable length
    float l = vmag(vsub(plPos, statePos));
    self->attachement_points[0].l = l; // assuming only 1 uav connected to payload
    struct vec a_indi = vzero();
    self->a_rpm_filtered = vzero();
    self->a_imu_filtered = vzero();
    //directional unit vector qi and its derivative qidot from UAV to payload
    self->qi = vnormalize(vsub(plPos, statePos));
    self->qidot = vdiv(vsub(plVel, stateVel),l);
    self->omega_c = vcross(self->qi, self->qidot); // cable angular velocity

    if ((self->indi & 1) && rpm_deck_available) {
      float f_rpm = t1 + t2 + t3 + t4;
      self->a_rpm = vsub(vsub(vscl(f_rpm / self->mass, mvmul(R, z)), gravity_comp), vscl(self->mp / self->mass, vadd(plAcc, gravity_comp)));
      update_butterworth_2_low_pass_vec(filter_acc_rpm, self->a_rpm);
      
      self->a_imu = mkvec(sensors->accNoLpf.x, sensors->accNoLpf.y, sensors->accNoLpf.z);
      self->a_imu = mvmul(R, self->a_imu); // to world frame
      self->a_imu.z-=1;
      self->a_imu = vscl(9.81, self->a_imu); // to SI unit (m/s^2)
      update_butterworth_2_low_pass_vec(filter_acc_imu, self->a_imu);

      self->a_rpm_filtered = get_butterworth_2_low_pass_vec(filter_acc_rpm);
      self->a_imu_filtered = get_butterworth_2_low_pass_vec(filter_acc_imu);
      a_indi = vsub(self->a_imu_filtered, self->a_rpm_filtered);
      // DEBUG
      // if (tick % 500 == 0) {
      //   DEBUG_PRINT("INDI p %f %f %f, %f %f %f\n", (double)self->a_rpm_filtered.x, (double)self->a_rpm_filtered.y, (double)self->a_rpm_filtered.z, (double)self->a_imu_filtered.x, (double)self->a_imu_filtered.y, (double)self->a_imu_filtered.z);
      // }
    }

    // payload desired force
    self->F_d = vscl(self->mp, 
      vadd5(
      plAcc_d, gravity_comp,
      veltmul(self->Kpos_P, plpos_e),
      veltmul(self->Kpos_D, plvel_e),
      veltmul(self->Kpos_I, self->i_error_pos)
    ));

    // cable desired force (desired virtual input), assuming only 1 uav connected to payload
    self->desVirtInp.x = self->F_d.x;
    self->desVirtInp.y = self->F_d.y;
    self->desVirtInp.z = self->F_d.z;

    struct mat33 qiqiT = vecmult(self->qi);
    // projection of the desired virtual input along qi
    struct vec virtualInp = mvmul(qiqiT, self->desVirtInp);
    struct vec u_parallel = vadd3(virtualInp, vscl(self->mass*l*vmag2(self->omega_c), self->qi), vscl(self->mass, mvmul(qiqiT, vdiv(self->desVirtInp, self->mp))));  

    // desired cable direction and its derivative
    struct mat33 skewqi = mcrossmat(self->qi); // skew symmetric matrix of qi
    struct mat33 skewqi2 = mmul(skewqi,skewqi); // skewqi squared

    // eq. 68-71 in Aggressive Maneuvering of a Quadrotor with a Cable-Suspended Payload by Sarah Tang
    self->qdi = vneg(vnormalize(vadd(plAcc_d, gravity_comp))); // reference cable direction
    self->omega_cd = vzero(); // reset omega_cd
    self->omega_cd_dot = vzero(); // reset omega_cd_dot
    float T = self->mp*vmag(vadd(plAcc_d, gravity_comp)); // cable tension
    float T_dot = -self->mp * vdot(self->plJerk_d, self->qdi); // tension derivative
    self->qdidot = vdiv(vneg(vadd(vscl(self->mp, self->plJerk_d), vscl(T_dot, self->qdi))), T);
    // T_ddot = -ml * (sl.dot(p) + jl.dot(p_dot))
    float T_ddot = -self->mp * (vdot(self->plSnap_d, self->qdi) + vdot(self->plJerk_d, self->qdidot));
    //  p_ddot = -(ml * sl + 2 * T_dot * p_dot + T_ddot * p) / T
    self->qddidot = vdiv(vneg(vadd3(vscl(self->mp, self->plSnap_d), vscl(2 * T_dot, self->qdidot), vscl(T_ddot, self->qdi))), T);
    // T_dddot = -ml * (dsl.dot(p) + 2 * sl.dot(p_dot) + jl.dot(p_ddot))
    float T_dddot = -self->mp * (vdot(self->pldSnap_d, self->qdi) +  2 * vdot(self->plSnap_d, self->qdidot)+ vdot(self->plJerk_d, self->qddidot));
    //  p_dddot = -(ml * dsl + 3 * T_ddot * p_dot + 3 * T_dot * p_ddot + T_dddot * p) / T
    self->qdddidot = vdiv(vneg(vadd4(vscl(self->mp, self->pldSnap_d), vscl(3 * T_ddot, self->qdidot), vscl(3 * T_dot, self->qddidot), vscl(T_dddot, self->qdi))), T);
    // omega_cdot =  ml*(np.cross(sl,p) + np.cross(jl,p_dot))
    if (self->use_flat_output & 1) {
      self->omega_cd = vcross(self->qdi, self->qdidot);         
      self->omega_cd_dot = vscl(self->mp, vadd(vcross(self->plSnap_d, self->qdi), vcross(self->plJerk_d, self->qdidot)));
    } else {
      self->omega_cd = vzero(); // reset omega_cd for now
      self->omega_cd_dot = vzero(); // reset omega_cd_dot for now
    }
    // cable error terms
    struct vec eq  = vclampscl(vcross(self->qdi, self->qi), -self->K_q_limit, self->K_q_limit);
    struct vec ew  = vclampscl(vadd(self->omega_c, mvmul(skewqi2, self->omega_cd)), -self->K_w_limit, self->K_w_limit);
    

    struct vec u_perpind = vsub(
      vscl(self->mass*l, 
        mvmul(skewqi,
          vadd4(
            vneg(veltmul(self->K_q, eq)),
            vneg(veltmul(self->K_w, ew)),
            vneg(vscl(vdot(self->qi, self->omega_cd), self->qidot)),
            vneg(mvmul(skewqi2, self->omega_cd_dot))
          )
        )
      ),
      vscl(self->mass, mvmul(skewqi2, vdiv(self->desVirtInp, self->mp)))
    );

    struct vec u_ctrl = vadd(u_parallel, u_perpind); // total desired force by the UAV on the cable
    //------------------------------------------------------------------------------------------//

    // UAV Lee controller
    struct vec pos_d = vsub(plPos_d, vscl(l, self->qdi)); // desired UAV position
    struct vec vel_d = vsub(plVel_d, vscl(l, self->qdidot)); // desired UAV velocity
    //logging payload state
    self->uav_pos_d = pos_d;
    self->uav_vel_d = vel_d;

    struct vec u_indi =  vscl(self->mass, a_indi);
    struct vec u = vsub(u_ctrl, u_indi); // add the PD control of the UAV to the feedforward force u
    float thrust_ctrl = vdot(u_ctrl, R_z);
    float thrust_indi = vdot(u_indi, R_z);
    control->thrustSi = thrust_ctrl - thrust_indi;
    self->thrustSi = control->thrustSi;
    
    // DEBUG PRINTS
    // static int counter = 0;
    // ++counter;
    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("thrust: %f\n", (double) self->thrustSi);
    //   DEBUG_PRINT("R_z: %f %f %f\n", (double) R_z.x, (double) R_z.y, (double) R_z.z);
    //   DEBUG_PRINT("u_par: %f %f %f\n", (double) u_parallel.x, (double) u_parallel.y, (double) u_parallel.z);
    //   DEBUG_PRINT("u_perpind: %f %f %f\n", (double) u_perpind.x, (double) u_perpind.y, (double) u_perpind.z);
    //   DEBUG_PRINT("length of the cable: %f\n", (double) l);
    //   DEBUG_PRINT("desired payload position: %f %f %f\n", (double) plPos_d.x, (double) plPos_d.y, (double) plPos_d.z);
    //   DEBUG_PRINT("payload position: %f %f %f\n", (double) plPos.x, (double) plPos.y, (double) plPos.z);
    //   DEBUG_PRINT("uav position: %f %f %f\n", (double) statePos.x, (double) statePos.y, (double) statePos.z);
    //   DEBUG_PRINT("desired uav position: %f %f %f\n", (double) pos_d.x, (double) pos_d.y, (double) pos_d.z);
    //   DEBUG_PRINT("desired uav velocity: %f %f %f\n", (double) vel_d.x, (double) vel_d.y, (double) vel_d.z);
    // }
    
    if (control->thrustSi < 0.01f) {
      controllerLeePayloadReset(self);
    }
    struct vec xb = vnormalize(vcross(yc, u));
    struct vec yb = vnormalize(vcross(u, xb));
    struct vec zb = vcross(xb, yb);
    self->R_des = mcolumns(xb, yb, zb); // desired rotation matrix of the UAV

  } else {
    if (setpoint->mode.z == modeDisable) {
      if (setpoint->thrust < 1000) {
          control->controlMode = controlModeForceTorque;
          control->thrustSi  = 0;
          control->torque[0] = 0;
          control->torque[1] = 0;
          control->torque[2] = 0;
          controllerLeePayloadReset(self);
          return;
      }
    }
    const float max_thrust = powerDistributionGetMaxThrust(); // N
    control->thrustSi = setpoint->thrust / UINT16_MAX * max_thrust;

    struct quat q = rpy2quat(mkvec(
        radians(setpoint->attitude.roll),
        -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
        desiredYaw));
    self->R_des = quat2rotmat(q);
  }

  // Lee attitude controller
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  self->rpy = quat2rpy(q);
  struct mat33 R = quat2rotmat(q);

  // desired rotation [Rdes]
  struct quat q_des = mat2quat(self->R_des);
  self->rpy_des = quat2rpy(q_des);

  // rotation error
  struct mat33 eRM = msub(mmul(mtranspose(self->R_des), R), mmul(mtranspose(R), self->R_des));

  struct vec eR = vscl(0.5f, mkvec(eRM.m[2][1], eRM.m[0][2], eRM.m[1][0]));

  // angular velocity
  self->omega = mkvec(
  radians(sensors->gyroNoLpf.x),
  radians(sensors->gyroNoLpf.y),
  radians(sensors->gyroNoLpf.z));

  struct vec desJerk = vsub(self->plJerk_d, vscl(self->attachement_points[0].l, self->qdddidot)); // desired UAV jerk 
  struct vec desSnap = vzero(); // desired UAV snap set to zero for now
  struct vec omega_des = vzero();

  // Compute desired omega NOT USED
  struct vec xb = mcolumn(self->R_des, 0);
  struct vec yb = mcolumn(self->R_des, 1);
  struct vec zb = mcolumn(self->R_des, 2);

  float c = control->thrustSi / (self->mass + self->mp);
  float B1 = c;
  float B3 = -vdot(yc, zb);
  float C3 = vmag(vcross(yc, zb));
  float D1 = vdot(xb, desJerk);
  float D2 = -vdot(yb, desJerk);
  float D3 = radians(setpoint->attitudeRate.yaw) * vdot(xc, xb);
  
  if (control->thrustSi != 0) {
    omega_des.x = D2/B1;
    omega_des.y = D1/B1;
    omega_des.z = (B1*D3-B3*D1)/(B1*C3);
  }

  // Compute desired omega dot
  float setpoint_yaw_ddot = radians(setpoint->attitudeAcc.yaw);
  float setpoint_yaw_dot  = radians(setpoint->attitudeRate.yaw);

  float c_dot = vdot(zb, desJerk);
  float E1 = vdot(xb,desSnap) - 2.0f * c_dot * omega_des.y - c * omega_des.x * omega_des.z;
  float E2 = -vdot(yb,desSnap) - 2.0f * c_dot * omega_des.x + c * omega_des.y * omega_des.z;
  float E3 = setpoint_yaw_ddot * vdot(xc, xb) + 2.0f * setpoint_yaw_dot * omega_des.z * vdot(xc, yb) - 2.0f * setpoint_yaw_dot*omega_des.y*vdot(xc,zb) - omega_des.x*omega_des.y*vdot(yc,yb) - omega_des.x*omega_des.z*vdot(yc,zb);

  self->omega_des_dot = vzero();
  if (control->thrustSi != 0) {
    self->omega_des_dot.x = E2/B1;
    self->omega_des_dot.y = E1/B1;
    self->omega_des_dot.z = (B1*E3-B3*E1)/(B1*C3);
  }

  self->omega_r = mvmul(mmul(mtranspose(R), self->R_des), omega_des);

  struct vec omega_error = vsub(self->omega, self->omega_r);
  
  // Integral part on angle
  self->i_error_att_uav = vadd(self->i_error_att_uav, vscl(dt, eR));


  self->tau = vadd5(
    vneg(veltmul(self->KR, eR)),
    vneg(veltmul(self->Komega, omega_error)),
    vneg(veltmul(self->KI, self->i_error_att_uav)),
    vcross(self->omega, veltmul(self->J, self->omega)),
    vneg(veltmul(self->J, vsub(mvmul(mcrossmat(self->omega), self->omega_r), mvmul(mmul(mtranspose(R), self->R_des), self->omega_des_dot)))));


  struct vec indi_moments = vzero();
  self->tau_rpm_filtered = vzero();
  self->tau_imu_filtered = vzero();
  if ((self->indi & 2) && rpm_deck_available) {
    const float t2t = 0.006f;
    const float arm = 0.707106781f * 0.046f;
    self->tau_rpm = mkvec(
      -arm * t1 - arm * t2 + arm * t3 + arm * t4,
      -arm * t1 + arm * t2 + arm * t3 - arm * t4,
      -t2t * t1 + t2t * t2 - t2t * t3 + t2t * t4
    );
    self->tau_rpm = vclampnorm(self->tau_rpm, 0.003);

    update_butterworth_2_low_pass_vec(filter_tau_rpm, self->tau_rpm);

    self->tau_rpm_filtered = get_butterworth_2_low_pass_vec(filter_tau_rpm);

    // angular accelleration
    uint64_t timestamp = usecTimestamp();
    float dt = (timestamp - self->timestamp_prev) / 1e6;
    struct vec omega_unfirltered = mkvec(radians(sensors->gyroNoLpf.x), radians(sensors->gyroNoLpf.y), radians(sensors->gyroNoLpf.z));
    struct vec angular_acc = vdiv(vsub(omega_unfirltered, self->omega_prev), dt);
    self->tau_imu = veltmul(self->J, angular_acc);
    self->tau_imu = vsub(self->tau_imu, vcross(veltmul(self->J, omega_unfirltered), omega_unfirltered));
    self->tau_imu = vclampnorm(self->tau_imu, 0.003); // rescale to avoid weird outliers

    update_butterworth_2_low_pass_vec(filter_tau_imu, self->tau_imu);

    self->tau_imu_filtered = get_butterworth_2_low_pass_vec(filter_tau_imu);
    self->omega_prev = omega_unfirltered;
    self->timestamp_prev = timestamp;

    indi_moments = vsub(self->tau_imu_filtered, self->tau_rpm_filtered);

    // DEBUG
    // if (tick % 1000 == 0) {
    //   DEBUG_PRINT("INDI a %f %f %f, %f %f %f\n", (double)self->tau_rpm_filtered.x, (double)self->tau_rpm_filtered.y, (double)self->tau_rpm_filtered.z, (double)self->tau_imu_filtered.x, (double)self->tau_imu_filtered.y, (double)self->tau_imu_filtered.z);
    // }
  }
  self->tau = vsub(self->tau, indi_moments);

  control->controlMode = controlModeForceTorque;
  control->torque[0] = self->tau.x;
  control->torque[1] = self->tau.y;
  control->torque[2] = self->tau.z;

}




#ifdef CRAZYFLIE_FW

#include "param.h"
#include "log.h"

void controllerLeePayloadFirmwareInit(void)
{
  controllerLeePayloadInit(&g_self);
}

bool controllerLeePayloadFirmwareTest(void)
{
  return true;
}

void controllerLeePayloadFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerLeePayload(&g_self, control, setpoint, sensors, state, tick);
}

PARAM_GROUP_START(ctrlLeeP)

// Payload Position P
PARAM_ADD(PARAM_FLOAT, Kpos_Px, &g_self.Kpos_P.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Py, &g_self.Kpos_P.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Pz, &g_self.Kpos_P.z)
PARAM_ADD(PARAM_FLOAT, Kpos_P_limit, &g_self.Kpos_P_limit)
// Payload Position D
PARAM_ADD(PARAM_FLOAT, Kpos_Dx, &g_self.Kpos_D.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Dy, &g_self.Kpos_D.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Dz, &g_self.Kpos_D.z)
PARAM_ADD(PARAM_FLOAT, Kpos_D_limit, &g_self.Kpos_D_limit)
// Payload Position I
PARAM_ADD(PARAM_FLOAT, Kpos_Ix, &g_self.Kpos_I.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Iy, &g_self.Kpos_I.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Iz, &g_self.Kpos_I.z)
PARAM_ADD(PARAM_FLOAT, Kpos_I_limit, &g_self.Kpos_I_limit)


// Cable P
PARAM_ADD(PARAM_FLOAT, Kqx, &g_self.K_q.x)
PARAM_ADD(PARAM_FLOAT, Kqy, &g_self.K_q.y)
PARAM_ADD(PARAM_FLOAT, Kqz, &g_self.K_q.z)
PARAM_ADD(PARAM_FLOAT, Kq_limit, &g_self.K_q_limit)

// Cable D
PARAM_ADD(PARAM_FLOAT, Kwx, &g_self.K_w.x)
PARAM_ADD(PARAM_FLOAT, Kwy, &g_self.K_w.y)
PARAM_ADD(PARAM_FLOAT, Kwz, &g_self.K_w.z)
PARAM_ADD(PARAM_FLOAT, Kw_limit, &g_self.K_w_limit)

// Cable I
PARAM_ADD(PARAM_FLOAT, KqIx, &g_self.K_q_I.x)
PARAM_ADD(PARAM_FLOAT, KqIy, &g_self.K_q_I.y)
PARAM_ADD(PARAM_FLOAT, KqIz, &g_self.K_q_I.z)

// UAV Position P
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Px, &g_self.Kpos_UAV_P.x)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Py, &g_self.Kpos_UAV_P.y)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Pz, &g_self.Kpos_UAV_P.z)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_P_limit, &g_self.Kpos_UAV_P_limit)
// UAV Position D
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Dx, &g_self.Kpos_UAV_D.x)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Dy, &g_self.Kpos_UAV_D.y)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Dz, &g_self.Kpos_UAV_D.z)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_D_limit, &g_self.Kpos_UAV_D_limit)
// UAV Position I
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Ix, &g_self.Kpos_UAV_I.x)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Iy, &g_self.Kpos_UAV_I.y)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Iz, &g_self.Kpos_UAV_I.z)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_I_limit, &g_self.Kpos_UAV_I_limit)


// UAV Attitude P
PARAM_ADD(PARAM_FLOAT, KRx, &g_self.KR.x)
PARAM_ADD(PARAM_FLOAT, KRy, &g_self.KR.y)
PARAM_ADD(PARAM_FLOAT, KRz, &g_self.KR.z)
PARAM_ADD(PARAM_FLOAT, KR_limit, &g_self.KR_limit)

// UAV Attitude D
PARAM_ADD(PARAM_FLOAT, Komx, &g_self.Komega.x)
PARAM_ADD(PARAM_FLOAT, Komy, &g_self.Komega.y)
PARAM_ADD(PARAM_FLOAT, Komz, &g_self.Komega.z)
PARAM_ADD(PARAM_FLOAT, Kom_limit, &g_self.Komega_limit)

// UAV Attitude I
PARAM_ADD(PARAM_FLOAT, KI_x, &g_self.KI.x)
PARAM_ADD(PARAM_FLOAT, KI_y, &g_self.KI.y)
PARAM_ADD(PARAM_FLOAT, KI_z, &g_self.KI.z)

// UAV and payload mass
PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)
PARAM_ADD(PARAM_FLOAT, massP, &g_self.mp)

// INDI status 
PARAM_ADD(PARAM_UINT8, indi, &g_self.indi)
PARAM_ADD(PARAM_UINT8, use_flat_output, &g_self.use_flat_output)

// Attachement points and cable lengths
PARAM_ADD(PARAM_UINT8, ap0id, &g_self.attachement_points[0].id)
PARAM_ADD(PARAM_FLOAT, ap0x, &g_self.attachement_points[0].point.x)
PARAM_ADD(PARAM_FLOAT, ap0y, &g_self.attachement_points[0].point.y)
PARAM_ADD(PARAM_FLOAT, ap0z, &g_self.attachement_points[0].point.z)
PARAM_ADD(PARAM_FLOAT, ap0l, &g_self.attachement_points[0].l)

PARAM_ADD(PARAM_UINT8, ap1id, &g_self.attachement_points[1].id)
PARAM_ADD(PARAM_FLOAT, ap1x, &g_self.attachement_points[1].point.x)
PARAM_ADD(PARAM_FLOAT, ap1y, &g_self.attachement_points[1].point.y)
PARAM_ADD(PARAM_FLOAT, ap1z, &g_self.attachement_points[1].point.z)
PARAM_ADD(PARAM_FLOAT, ap1l, &g_self.attachement_points[1].l)

PARAM_ADD(PARAM_UINT8, ap2id, &g_self.attachement_points[2].id)
PARAM_ADD(PARAM_FLOAT, ap2x, &g_self.attachement_points[2].point.x)
PARAM_ADD(PARAM_FLOAT, ap2y, &g_self.attachement_points[2].point.y)
PARAM_ADD(PARAM_FLOAT, ap2z, &g_self.attachement_points[2].point.z)
PARAM_ADD(PARAM_FLOAT, ap2l, &g_self.attachement_points[2].l)

PARAM_GROUP_STOP(ctrlLeeP)


LOG_GROUP_START(ctrlLeeP)

LOG_ADD(LOG_FLOAT, thrustSI, &g_self.thrustSi)
LOG_ADD(LOG_FLOAT, torquex, &g_self.tau.x)
LOG_ADD(LOG_FLOAT, torquey, &g_self.tau.y)
LOG_ADD(LOG_FLOAT, torquez, &g_self.tau.z)

// current angles
LOG_ADD(LOG_FLOAT, rpyx, &g_self.rpy.x)
LOG_ADD(LOG_FLOAT, rpyy, &g_self.rpy.y)
LOG_ADD(LOG_FLOAT, rpyz, &g_self.rpy.z)

// desired angles
LOG_ADD(LOG_FLOAT, rpydx, &g_self.rpy_des.x)
LOG_ADD(LOG_FLOAT, rpydy, &g_self.rpy_des.y)
LOG_ADD(LOG_FLOAT, rpydz, &g_self.rpy_des.z)


// omega
LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

LOG_ADD(LOG_FLOAT, uav_posdx, &g_self.uav_pos_d.x)
LOG_ADD(LOG_FLOAT, uav_posdy, &g_self.uav_pos_d.y)
LOG_ADD(LOG_FLOAT, uav_posdz, &g_self.uav_pos_d.z)

LOG_ADD(LOG_FLOAT, uav_veldx, &g_self.uav_vel_d.x)
LOG_ADD(LOG_FLOAT, uav_veldy, &g_self.uav_vel_d.y)
LOG_ADD(LOG_FLOAT, uav_veldz, &g_self.uav_vel_d.z)

// Cable states 
LOG_ADD(LOG_FLOAT, qix, &g_self.qi.x)
LOG_ADD(LOG_FLOAT, qiy, &g_self.qi.y)
LOG_ADD(LOG_FLOAT, qiz, &g_self.qi.z)

LOG_ADD(LOG_FLOAT, qidotx, &g_self.qidot.x)
LOG_ADD(LOG_FLOAT, qidoty, &g_self.qidot.y)
LOG_ADD(LOG_FLOAT, qidotz, &g_self.qidot.z)

LOG_ADD(LOG_FLOAT, qdix, &g_self.qdi.x)
LOG_ADD(LOG_FLOAT, qdiy, &g_self.qdi.y)
LOG_ADD(LOG_FLOAT, qdiz, &g_self.qdi.z)

LOG_ADD(LOG_FLOAT, qdidotx, &g_self.qdidot.x)
LOG_ADD(LOG_FLOAT, qdidoty, &g_self.qdidot.y)
LOG_ADD(LOG_FLOAT, qdidotz, &g_self.qdidot.z)

// INDI
LOG_ADD(LOG_FLOAT, tau_rpmx, &g_self.tau_rpm.x)  // compare to torquex
LOG_ADD(LOG_FLOAT, tau_rpmy, &g_self.tau_rpm.y)  // compare to torquey
LOG_ADD(LOG_FLOAT, tau_rpmz, &g_self.tau_rpm.z)  // compare to torquez

LOG_ADD(LOG_FLOAT, tau_rpm_fx, &g_self.tau_rpm_filtered.x)  // compare to torquex
LOG_ADD(LOG_FLOAT, tau_rpm_fy, &g_self.tau_rpm_filtered.y)  // compare to torquey
LOG_ADD(LOG_FLOAT, tau_rpm_fz, &g_self.tau_rpm_filtered.z)  // compare to torquez

LOG_ADD(LOG_FLOAT, tau_imu_x, &g_self.tau_imu.x)  // compare to torquex
LOG_ADD(LOG_FLOAT, tau_imu_y, &g_self.tau_imu.y)  // compare to torquey
LOG_ADD(LOG_FLOAT, tau_imu_z, &g_self.tau_imu.z)  // compare to torquez

LOG_ADD(LOG_FLOAT, tau_imu_fx, &g_self.tau_imu_filtered.x)  // compare to torquex
LOG_ADD(LOG_FLOAT, tau_imu_fy, &g_self.tau_imu_filtered.y)  // compare to torquey
LOG_ADD(LOG_FLOAT, tau_imu_fz, &g_self.tau_imu_filtered.z)  // compare to torquez

LOG_ADD(LOG_FLOAT, a_rpmx, &g_self.a_payload.x)
LOG_ADD(LOG_FLOAT, a_rpmy, &g_self.a_payload.y)
LOG_ADD(LOG_FLOAT, a_rpmz, &g_self.a_payload.z)

LOG_ADD(LOG_FLOAT, a_rpm_fx, &g_self.a_rpm_filtered.x)
LOG_ADD(LOG_FLOAT, a_rpm_fy, &g_self.a_rpm_filtered.y)
LOG_ADD(LOG_FLOAT, a_rpm_fz, &g_self.a_rpm_filtered.z)

LOG_ADD(LOG_FLOAT, a_imux, &g_self.a_imu.x)
LOG_ADD(LOG_FLOAT, a_imuy, &g_self.a_imu.y)
LOG_ADD(LOG_FLOAT, a_imuz, &g_self.a_imu.z)

LOG_ADD(LOG_FLOAT, a_imu_fx, &g_self.a_imu_filtered.x)
LOG_ADD(LOG_FLOAT, a_imu_fy, &g_self.a_imu_filtered.y)
LOG_ADD(LOG_FLOAT, a_imu_fz, &g_self.a_imu_filtered.z)


LOG_GROUP_STOP(ctrlLeeP)

#endif // CRAZYFLIE_FW defined
