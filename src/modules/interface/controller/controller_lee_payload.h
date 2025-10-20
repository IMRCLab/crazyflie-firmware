/*
The MIT License (MIT)

Copyright (c) 2025 IMRC LAB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef __CONTROLLER_LEE_PAYLOAD__
#define __CONTROLLER_LEE_PAYLOAD__

#include "stabilizer_types.h"
#include "math3d.h"

typedef struct controllerLeePayload_s {
    float mass;
    float mp; // payload mass
    struct {
        uint8_t id;
        struct vec point;
        float l; // cable length; set to <= 0 to compute automatically based on the measurements
    } attachement_points[MAX_TEAM_SIZE];
    float thrustSi;
    struct vec J; // Inertia matrix (diagonal matrix); kg m^2

    struct vec plJerk_d; // desired payload jerk
    struct vec plSnap_d; // desired payload snap
    struct vec pldSnap_d; // desired payload snap derivative
    struct vec a_cl; // desired acc + closed-loop (PID) 
    // Payload PID
    struct vec Kpos_P; // Payload Kp 
    float Kpos_P_limit;
    struct vec Kpos_D; // Payload Kv 
    float Kpos_D_limit;
    struct vec Kpos_I; // Payload Ki 
    float Kpos_I_limit;
    struct vec i_error_pos; // integral of payload position error

   // Cable PD 
    struct vec K_q; // cable direction P
    float K_q_limit;
    struct vec K_w; // cable angular velocity D
    float K_w_limit;
    struct vec K_q_I; // cable direction I
    struct vec i_error_q;  // integral of cable direction error
    uint8_t use_flat_output; // use flat output for payload and cable control

    // UAV Position PID Gains
    struct vec Kpos_UAV_P; // UAV position Kp
    float Kpos_UAV_P_limit;
    struct vec Kpos_UAV_D; // UAV position Kv
    float Kpos_UAV_D_limit;
    struct vec Kpos_UAV_I; // UAV position Ki
    float Kpos_UAV_I_limit;
    struct vec i_error_pos_uav; // integral of UAV position error

    
    // UAV Attitude PID
    struct vec KR; // UAV rotation KR
    float KR_limit;
    struct vec Komega; // UAV omega Komega
    float Komega_limit;
    struct vec KI; // UAV attitude integral KI
    struct vec i_error_att_uav; // integral of attitude error


    // Payload and Cable controller components (also used for logging)    
    struct vec F_d; // desired payload force
    struct vec desVirtInp; // desired cable force
    
    struct vec qi; // cable direction
    struct vec qidot;  //  cable direction derivative
    struct vec omega_c; // cable angular velocity

    struct vec qdi;    // desired cable desired direction -> desVirtInp normalized
    struct vec qi_ref;    // desired cable desired direction -> from desired acc + gravity
    struct vec qdidot; // desired cable direction derivative
    struct vec qddidot; // desired cable direction 2nd derivative
    struct vec qdddidot; // desired cable direction 3rd derivative
    struct vec omega_cd; // desired cable angular velocity
    struct vec omega_cd_dot; // desired cable angular acceleration

    // INDI
    uint8_t indi;
    struct vec omega_prev;
    uint64_t timestamp_prev;

    struct vec tau_rpm;
    struct vec tau_rpm_filtered;
    struct vec tau_imu_filtered;
    struct vec tau_imu;

    struct vec a_payload;
    struct vec a_rpm;
    struct vec a_rpm_filtered;
    struct vec a_imu;
    struct vec a_imu_filtered;

    // Logging variables
    struct vec uav_pos_d;
    struct vec uav_vel_d;
    struct vec rpy;
    struct vec rpy_des;
    struct mat33 R_des;
    struct vec omega;
    struct vec omega_r;
    struct vec tau;
    struct vec omega_des_dot;
} controllerLeePayload_t;


void controllerLeePayloadInit(controllerLeePayload_t* self);
void controllerLeePayloadReset(controllerLeePayload_t* self);
void controllerLeePayload(controllerLeePayload_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#ifdef CRAZYFLIE_FW

void controllerLeePayloadFirmwareInit(void);
bool controllerLeePayloadFirmwareTest(void);
void controllerLeePayloadFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif // CRAZYFLIE_FW

#endif //__CONTROLLER_LEE_PAYLOAD__
