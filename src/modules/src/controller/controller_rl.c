/*
The MIT License (MIT)

Copyright (c) 2024 IMRC Lab

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

// For STAI docs see: https://stedgeai-dc.st.com/assets/embedded-docs/

#include <math.h>
#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#include "math3d.h"
#include "controller_lee.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "usec_time.h"
#include "motors.h"

#include "network.h"  // Generated model definitions (activations, weights, I/O sizes)
#include "network_data.h"

/* Global handle to reference the instantiated C-model */
static ai_handle network = AI_HANDLE_NULL;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
static ai_float in_data[AI_NETWORK_IN_1_SIZE];
/* or static ai_u8 in_data[AI_NETWORK_IN_1_SIZE_BYTES]; */

/* c-array to store the data of the output tensor */
AI_ALIGNED(32)
static ai_float out_data[AI_NETWORK_OUT_1_SIZE];
/* static ai_u8 out_data[AI_NETWORK_OUT_1_SIZE_BYTES]; */

/* Array of pointer to manage the model's input/output tensors */
static ai_buffer *ai_input;
static ai_buffer *ai_output;


static float lastAction[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
// static uint32_t rl_print_counter = 0;

// #define THRUST_MIN 0.0f     // Minimum thrust (N)
// #define THRUST_MAX 0.118f   // Maximum thrust (N)


/* 
 * Bootstrap
 */
int aiInit(void) {
  ai_error err;
  
  /* Create and initialize the c-model */
  const ai_handle acts[] = { activations };
  err = ai_network_create_and_init(&network, acts, NULL);
  if (err.type != AI_ERROR_NONE) { 
    DEBUG_PRINT("AI network creation failed with code: %d\n", err.type);
    return -1;
   };

  /* Reteive pointers to the model's input/output tensors */
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);

  return 0;
}

void controllerRLFirmwareInit(void)
{
  int s = aiInit();
  if (s != 0) {
    DEBUG_PRINT("AI initialization failed with code: %d\n", s);
    return;
  }

  // // initialize last action
  // for (uint32_t id = 0; id < 4; ++id) {
  //   uint16_t pwm = motorsGetRatio(id);

  //   // thrust = a * pwm^2 + b * pwm
  //   //    where PWM is normalized (range 0...1)
  //   //          thrust is in Newtons (per rotor)
  //   float pwmToThrustA = 0.091492681f;
  //   float pwmToThrustB = 0.067673604f;

  //   float pwm_normalized = pwm / UINT16_MAX;
  //   float thrust_in_newton = pwmToThrustA * pwm_normalized * pwm_normalized + pwmToThrustB * pwm_normalized;
  //   lastAction[id] = (thrust_in_newton / 0.118f - 0.5f) * 2.0f;

  // }
}

void controllerRLPayloadFirmwareInit(void)
{
  controllerRLFirmwareInit();
  // TODO: can set a flag here, if needed
}


/* 
 * Run inference
 */
int aiRun(const void *in_data, void *out_data) {
  ai_i32 n_batch;
  ai_error err;
  
  /* 1 - Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(in_data);
  ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* 2 - Perform the inference */
  n_batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
  if (n_batch != 1) {
      err = ai_network_get_error(network);
      DEBUG_PRINT("AI network run failed with code: %d\n", err.type);
      return -1;
  };
  
  return 0;
}

void controllerRLFirmware(control_t *control,
                          const setpoint_t *setpoint,
                          const sensorData_t *sensors,
                          const state_t     *state,
                          const uint32_t     tick)
{
  if (!RATE_DO_EXECUTE(250, tick)) return;

  /*— compute body-frame errors, velocities, rotation matrix —*/
  struct quat q        = qnormalize(mkquat(state->attitudeQuaternion.x,
                                state->attitudeQuaternion.y,
                                state->attitudeQuaternion.z,
                                state->attitudeQuaternion.w));
  struct mat33 R       = quat2rotmat(q);
  struct vec vel_world = mkvec(state->velocity.x,
                               state->velocity.y,
                               state->velocity.z);
  // struct vec vel_body  = qvrot(qinv(q), vel_world);
  // struct vec pos_des   = mkvec(setpoint->position.x,
  //                               setpoint->position.y,
  //                               setpoint->position.z);
  struct vec pos_des   = mkvec( 0.0f, 0.0f, 1.0f);
  struct vec pos       = mkvec(state->position.x,
                                state->position.y,
                                state->position.z);

  struct vec payload_pos = mkvec(state->payload_pos.x,
                                state->payload_pos.y,
                                state->payload_pos.z);

  struct vec payload_vel_world = mkvec(state->payload_vel.x,
                               state->payload_vel.y,
                               state->payload_vel.z);

  struct vec pos_err   = vsub(pos_des, pos);
  struct vec pos_err_clamped = vclampnorm(pos_err, 1.0f);

  struct vec payload_pos_err   = vsub(pos_des, payload_pos);
  struct vec payload_pos_err_clamped = vclampnorm(payload_pos_err, 1.0f);

  struct vec rel_pos = vsub(pos, payload_pos);

  struct vec other_pos_1 = mkvec(state->team_state[1].pos.x, state->team_state[1].pos.y, state->team_state[1].pos.z);
  struct vec other_pos_2 = mkvec(state->team_state[2].pos.x, state->team_state[2].pos.y, state->team_state[2].pos.z);

  struct vec rel_pos_other_1 = vsub(other_pos_1, payload_pos);
  struct vec rel_pos_other_2 = vsub(other_pos_2, payload_pos);

  // // rotate state acceleration in G to body frrame
  // struct vec acc_world = mkvec(state->acc.x, state->acc.y, state->acc.z);
  // struct vec acc_body  = qvrot(qinv(q), acc_world);
 
  // raw accel in body frame (Gs)
  // struct vec raw_body = mkvec(sensors->acc.x, sensors->acc.y, sensors->acc.z);
  // // convert to world-frame (Gs) via R, then to m/s^2 and subtract 1g
  // struct vec acc_world = {
  //   R.m[0][0]*raw_body.x + R.m[0][1]*raw_body.y + R.m[0][2]*raw_body.z,
  //   R.m[1][0]*raw_body.x + R.m[1][1]*raw_body.y + R.m[1][2]*raw_body.z,
  //   R.m[2][0]*raw_body.x + R.m[2][1]*raw_body.y + R.m[2][2]*raw_body.z
  // };
  // acc_world.x *= 9.80665f;
  // acc_world.y *= 9.80665f;
  // acc_world.z = (acc_world.z - 1.0f) * 9.80665f;
  // // rotate back to body frame (m/s^2)
  // struct vec acc_body = qvrot(qinv(q), acc_world);

  // struct vec omega_body = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)); // rad/s
  // struct vec omega_world = qvrot(q, omega_body); // rad/s

  // struct quat q_dot = qscl(0.5f, qqmul2(q, quatvw(omega_body, 0.0f)));

  /*— fill input array —*/
  in_data[0]  = payload_pos_err_clamped.x;
  in_data[1]  = payload_pos_err_clamped.y;
  in_data[2]  = payload_pos_err_clamped.z;
  in_data[3]  = payload_vel_world.x;
  in_data[4]  = payload_vel_world.y;
  in_data[5]  = payload_vel_world.z;
  in_data[6]  = rel_pos.x; // rel_pos is 0 for no payload
  in_data[7]  = rel_pos.y; // rel_vel is
  in_data[8]  = rel_pos.z; // rel_acc is
  in_data[9]  = R.m[0][0];
  in_data[10] = R.m[0][1];
  in_data[11] = R.m[0][2];
  in_data[12] = R.m[1][0];
  in_data[13] = R.m[1][1];
  in_data[14] = R.m[1][2];
  in_data[15] = R.m[2][0];
  in_data[16] = R.m[2][1];
  in_data[17] = R.m[2][2];
  in_data[18] = vel_world.x; // linvels 0.0f for no payload
  in_data[19] = vel_world.y; // linvels 0.0f for no payload
  in_data[20] = vel_world.z; // linvels 0.0f for no payload
  in_data[21] = radians(sensors->gyroNoLpf.x);
  in_data[22] = radians(sensors->gyroNoLpf.y);
  in_data[23] = radians(sensors->gyroNoLpf.z);
  in_data[24] = lastAction[0];
  in_data[25] = lastAction[1];
  in_data[26] = lastAction[2];
  in_data[27] = lastAction[3];
  in_data[28] = rel_pos_other_1.x;
  in_data[29] = rel_pos_other_1.y;
  in_data[30] = rel_pos_other_1.z;
  in_data[31] = rel_pos_other_2.x;
  in_data[32] = rel_pos_other_2.y;
  in_data[33] = rel_pos_other_2.z;

    /* 2 - Call inference engine */
    aiRun(in_data, out_data);
    control->controlMode = controlModeForce;
    for (int i = 0; i < 4; i++) {
        float a = out_data[i];
        if (a >  1.0f) a =  1.0f;
        if (a < -1.0f) a = -1.0f;
        lastAction[i] = a;
        control->normalizedForces[i] = 0.5f * (a + 1.0f);
    }

    // if (++rl_print_counter % 100 == 0) {
    //     DEBUG_PRINT("action = [%f, %f, %f, %f]\n",
    //       out_data[0], out_data[1],
    //       out_data[2], out_data[3]);
    //     rl_print_counter = 0;
    // }

    
}

bool controllerRLFirmwareTest(void)
{
  return true;
}

LOG_GROUP_START(ctrlrl)
LOG_ADD_CORE(LOG_FLOAT, in0, &in_data[0])
LOG_ADD_CORE(LOG_FLOAT, in1, &in_data[1])
LOG_ADD_CORE(LOG_FLOAT, in2, &in_data[2])
LOG_ADD_CORE(LOG_FLOAT, in3, &in_data[3])
LOG_ADD_CORE(LOG_FLOAT, in4, &in_data[4])
LOG_ADD_CORE(LOG_FLOAT, in5, &in_data[5])
LOG_ADD_CORE(LOG_FLOAT, in6, &in_data[6])
LOG_ADD_CORE(LOG_FLOAT, in7, &in_data[7])
LOG_ADD_CORE(LOG_FLOAT, in8, &in_data[8])
LOG_ADD_CORE(LOG_FLOAT, in9, &in_data[9])
LOG_ADD_CORE(LOG_FLOAT, in10, &in_data[10])
LOG_ADD_CORE(LOG_FLOAT, in11, &in_data[11])
LOG_ADD_CORE(LOG_FLOAT, in12, &in_data[12])
LOG_ADD_CORE(LOG_FLOAT, in13, &in_data[13])
LOG_ADD_CORE(LOG_FLOAT, in14, &in_data[14])
LOG_ADD_CORE(LOG_FLOAT, in15, &in_data[15])
LOG_ADD_CORE(LOG_FLOAT, in16, &in_data[16])
LOG_ADD_CORE(LOG_FLOAT, in17, &in_data[17])
LOG_ADD_CORE(LOG_FLOAT, in18, &in_data[18])
LOG_ADD_CORE(LOG_FLOAT, in19, &in_data[19])
LOG_ADD_CORE(LOG_FLOAT, in20, &in_data[20])
LOG_ADD_CORE(LOG_FLOAT, in21, &in_data[21])
LOG_ADD_CORE(LOG_FLOAT, in22, &in_data[22])
LOG_ADD_CORE(LOG_FLOAT, in23, &in_data[23])
LOG_ADD_CORE(LOG_FLOAT, in24, &in_data[24])
LOG_ADD_CORE(LOG_FLOAT, in25, &in_data[25])
LOG_ADD_CORE(LOG_FLOAT, in26, &in_data[26])
LOG_ADD_CORE(LOG_FLOAT, in27, &in_data[27])
LOG_ADD_CORE(LOG_FLOAT, in28, &in_data[28])
LOG_ADD_CORE(LOG_FLOAT, in29, &in_data[29])
LOG_ADD_CORE(LOG_FLOAT, in30, &in_data[30])

LOG_ADD_CORE(LOG_FLOAT, out0, &out_data[0])
LOG_ADD_CORE(LOG_FLOAT, out1, &out_data[1])
LOG_ADD_CORE(LOG_FLOAT, out2, &out_data[2])
LOG_ADD_CORE(LOG_FLOAT, out3, &out_data[3])
LOG_GROUP_STOP(ctrlrl)