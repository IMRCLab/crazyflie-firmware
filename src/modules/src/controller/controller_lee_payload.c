/*
The MIT License (MIT)

Copyright (c) 2022 Khaled Wahba 

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

/*
This controller is based on the following publication:

TODO
*/

#include <math.h>
#include <string.h>
#include "math3d.h"
#include "controller_lee_payload.h"
#include "stdio.h"
#include "debug.h"
#include "usec_time.h"
#include "filter.h"

#include "log.h"
#include "param.h"

#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"
// QP
// #include "workspace_2uav_2hp.h"
#include "osqp.h"
#include "scaling.h"
#include "auxil.h"

#include "nn.h"
#include "usec_time.h"
float nn_inference_time_payload;

extern OSQPWorkspace workspace_2uav_2hp;
extern OSQPWorkspace workspace_3uav_2hp;
extern OSQPWorkspace workspace_4uav_5hp;
// extern OSQPWorkspace workspace_5uav_9hp;
// extern OSQPWorkspace workspace_6uav_14hp;
// extern OSQPWorkspace workspace_7uav_42hp;
// extern OSQPWorkspace workspace_8uav_56hp;
// extern OSQPWorkspace workspace_9uav_72hp;
// extern OSQPWorkspace workspace_10uav_90hp;
// extern OSQPWorkspace workspace_2uav_1hp_rod  ;
// extern OSQPWorkspace workspace_3uav_6hp_rig  ;
// extern OSQPWorkspace workspace_4uav_12hp_rig ;
// extern OSQPWorkspace workspace_5uav_20hp_rig ;
// extern OSQPWorkspace workspace_6uav_30hp_rig ;
// extern OSQPWorkspace workspace_7uav_42hp_rig ;
// extern OSQPWorkspace workspace_8uav_56hp_rig ;
// extern OSQPWorkspace workspace_9uav_72hp_rig ;
// extern OSQPWorkspace workspace_10uav_90hp_rig;
extern OSQPWorkspace workspace_hyperplane;
extern OSQPWorkspace workspace_hyperplane_rb;
extern OSQPWorkspace workspace_compute_Fd_pair;

#define GRAVITY_MAGNITUDE (9.81f)


struct QPInput
{
  struct vec F_d;
  struct vec M_d;

  struct vec plStPos;
  struct quat plStquat;

  uint8_t num_uavs;
  struct {
    uint8_t id;
    struct vec pos;
    struct vec attPoint;
    struct vec mu_ref; // from the planner
    float l;
  } team_state[MAX_TEAM_SIZE];

  controllerLeePayload_t* self;
  uint32_t timestamp; // ticks, i.e., ms
};

struct QPOutput
{
  struct vec desVirtInp;
  uint32_t timestamp; // ticks (copied from input)
  bool success;
};

static uint64_t time_start;


#ifdef CRAZYFLIE_FW

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "static_mem.h"
#include "eventtrigger.h"


#define CONTROLLER_LEE_PAYLOAD_QP_TASK_STACKSIZE (7 * configMINIMAL_STACK_SIZE)
#define CONTROLLER_LEE_PAYLOAD_QP_TASK_NAME "LEEQP"
#define CONTROLLER_LEE_PAYLOAD_QP_TASK_PRI 0

STATIC_MEM_TASK_ALLOC(controllerLeePayloadQPTask, CONTROLLER_LEE_PAYLOAD_QP_TASK_STACKSIZE);
static void controllerLeePayloadQPTask(void * prm);
static bool taskInitialized = false;

static QueueHandle_t queueQPInput;
STATIC_MEM_QUEUE_ALLOC(queueQPInput, 1, sizeof(struct QPInput));
static QueueHandle_t queueQPOutput;
STATIC_MEM_QUEUE_ALLOC(queueQPOutput, 1, sizeof(struct QPOutput));

static uint32_t qp_runtime_us = 0;

// all times are in microseconds / 8 (i.e., up to 64*8ms can be measured)
EVENTTRIGGER(qpSolved, uint16, Fd, uint16, svm, uint16, mu, uint16, total)

#else
#include <time.h>

void print_csc_matrix(csc *M, const char *name)
{
  c_int j, i, row_start, row_stop;
  c_int k = 0;

  // Print name
  printf("%s :\n", name);

  for (j = 0; j < M->n; j++) {
    row_start = M->p[j];
    row_stop  = M->p[j + 1];

    if (row_start == row_stop) continue;
    else {
      for (i = row_start; i < row_stop; i++) {
        printf("\t[%3u,%3u] = %.3g\n", (int)M->i[i], (int)j, M->x[k++]);
      }
    }
  }
}

void print_dns_matrix(c_float *M, c_int m, c_int n, const char *name)
{
  c_int i, j;

  printf("%s : \n\t", name);

  for (i = 0; i < m; i++) {   // Cycle over rows
    for (j = 0; j < n; j++) { // Cycle over columns
      if (j < n - 1)
        // c_print("% 14.12e,  ", M[j*m+i]);
        printf("% .3g,  ", M[j * m + i]);

      else
        // c_print("% 14.12e;  ", M[j*m+i]);
        printf("% .3g;  ", M[j * m + i]);
    }

    if (i < m - 1) {
      printf("\n\t");
    }
  }
  printf("\n");
}

void print_vec(c_float *v, c_int n, const char *name) {
  print_dns_matrix(v, 1, n, name);
}

#endif

// helper function similar as first part of osqp_update_A and osqp_update_P
// to make updating entries more time and memory efficient
static void osqp_prepare_update(OSQPWorkspace *work)
{
  if (work->settings->scaling) {
    // Unscale data
    unscale_data(work);
  }
}

// helper function similar as first part of osqp_update_A and osqp_update_P
// to make updating entries more time and memory efficient
static c_int osqp_finalize_update(OSQPWorkspace *work)
{
  c_int exitflag; // Exit flag

  if (work->settings->scaling) {
    // Scale data
    scale_data(work);
  }

  // Update linear system structure with new data
  exitflag = work->linsys_solver->update_matrices(work->linsys_solver,
                                                  work->data->P,
                                                  work->data->A);

  // Reset solver information
  reset_info(work->info);

  return exitflag;
}

static inline struct vec computePlaneNormal(struct vec ps1, struct vec ps2, struct vec pload, float r, float l1, float l2) {
// Compute the normal of a plane, given the minimum desired distance r
// NEEDs TESTING!!!
  struct vec p1 = ps1;
  struct vec p2 = ps2;
  if (l1 <= l2) {
    p1 = ps1;
    p2 = ps2;
  }
  else {
    p1 = ps2;
    p2 = ps1;
  }
  struct vec p1_r = vsub(p1, pload);
  struct vec p2_r = vsub(p2, pload);

  float p1_dot_p2 = vdot(p1_r, p2_r);
  float normp1    = vmag(p1_r);
  float normp2    = vmag(p2_r);
  
  float angle_12  = acosf(p1_dot_p2/(normp1*normp2));
  struct vec p1_r_proj = mkvec(p1_r.x, p1_r.y, 0);
  float normp1_r_proj  = vmag(p1_r_proj);
  float angle_1 = M_PI_2_F;
  if (normp1_r_proj > 0) {
     angle_1 = acosf((vdot(p1_r, p1_r_proj))/(normp1*normp1_r_proj));
  }

  float angle_2 = M_PI_F - (angle_12 + angle_1);
  float normp2_r_new = normp1 * (sinf(angle_1)/sinf(angle_2));

  struct vec p2_new = vadd(pload, vscl(normp2_r_new, vnormalize(p2_r)));
  struct vec pos1 = ps1;
  struct vec pos2 = ps2;
  if(l2 >= l1) {
    pos2 = p2_new;
  }
  else{
    pos1 = p2_new;
  }

  struct vec mid = vscl(0.5, vsub(pos2, pos1));
  struct vec rvec = vscl(r, vnormalize(vsub(pos1, pos2)));
  struct vec pr = vadd(pos1, vadd(mid, rvec));

  struct vec p0pr = vsub(pr, pload);
  struct vec prp2 = vsub(pos2, pr);
  struct vec ns = vcross(prp2, p0pr);
  struct vec n_sol = vcross(p0pr, ns);
  return n_sol;
}

static inline void computePlaneNormals(struct vec p1, struct vec p2, struct vec pload, float r, float l1, float l2, float lambda_svm, struct vec Fd, struct vec* n1, struct vec* n2,
  struct osqp_warmstart_hyperplane* warmstart_state, float* solve_time_svm) {
  // relative coordinates
  struct vec p1_r = vsub(p1, pload);
  struct vec p2_r = vsub(p2, pload);

  // run QP to find separating hyperplane
  OSQPWorkspace* workspace = &workspace_hyperplane;

  osqp_prepare_update(workspace);

  // Update P
  workspace->data->P->x[3] = 2 * lambda_svm;

  // Update A
  workspace->data->A->x[0] = Fd.x;
  workspace->data->A->x[1] = p1_r.x;
  workspace->data->A->x[2] = -p2_r.x;
  workspace->data->A->x[3] = Fd.y;
  workspace->data->A->x[4] = p1_r.y;
  workspace->data->A->x[5] = -p2_r.y;
  workspace->data->A->x[6] = Fd.z;
  workspace->data->A->x[7] = p1_r.z;
  workspace->data->A->x[8] = -p2_r.z;

  osqp_finalize_update(workspace);

  osqp_warm_start(workspace, warmstart_state->x, warmstart_state->y);

  #ifdef CRAZYFLIE_FW
    uint64_t timestamp_svm_start = usecTimestamp();
  #else
    clock_t timestamp_svm_start = clock();
  #endif
  osqp_solve(workspace);
  #ifdef CRAZYFLIE_FW
    eventTrigger_qpSolved_payload.svm += (usecTimestamp() - timestamp_svm_start) / 8;
  #else
    *solve_time_svm += ((float)(clock() - timestamp_svm_start)) / CLOCKS_PER_SEC * 1000.0f;
  #endif

  if (workspace->info->status_val == OSQP_SOLVED) {
    memcpy(warmstart_state->x, workspace->solution->x, sizeof(warmstart_state->x));
    memcpy(warmstart_state->y, workspace->solution->y, sizeof(warmstart_state->y));

    struct vec n = vloadf(workspace->solution->x);

    // now rotate the hyperplane in two directions to compute the two resulting hyperplanes
    struct vec axis = vcross(n, mkvec(0,0,1));

    if (r > 2.0f * l1) {
      *n1 = n;
    } else {
      float angle1 = 2 * asinf(r / (2.0f * l1));
      struct quat q1 = qaxisangle(axis, angle1);
      *n1 = qvrot(q1, n);
    }

    if (r > 2.0f * l2) {
      *n2 = n;
    } else {
      float angle2 = 2 * asinf(r / (2.0f * l2));
      struct quat q2 = qaxisangle(axis, -angle2);
      *n2 = vneg(qvrot(q2, n));
    }
  } else {
  #ifdef CRAZYFLIE_FW
        DEBUG_PRINT("QPsvm: %s\n", workspace->info->status);
  #else
        printf("QPsvm: %s\n", workspace->info->status);
  #endif

    *n1 = vzero();
    *n2 = vzero();
  }
}

// Computes the circle that is created when intersecting a plane given as (n.p = a)
// with a sphere at center c with radius r
// See https://math.stackexchange.com/questions/943383/determine-circle-of-intersection-of-plane-and-sphere
static void plane_sphere_intersection(struct vec n, float a, struct vec c, float r, struct vec* result_center, float* result_radius)
{
  // compute signed distance sphere -> hyperplane
  float length = vmag(n);
  float dist = (vdot(n, c) - a) / length;

  // if the minimum distance is greater than r, there is no intersection
  if (fabsf(dist) > r) {
    *result_center = vzero();
    *result_radius = -1;
    return;
  }
  // otherwise, compute the center point (on the plane) and radius of the resulting circle
  *result_center = vsub(c, vscl(dist / length, n));
  *result_radius = sqrtf(r*r - dist*dist);
}

// projects point p onto plane (given by n.p = a)
static struct vec project_point_on_plane(struct vec n, float a, struct vec p)
{
    // compute signed distance sphere -> hyperplane
    float length = vmag(n);
    float dist = (vdot(n, p) - a) / length;

    struct vec p_on_plane = vsub(p, vscl(dist / length, n));
    return p_on_plane;
}

static inline void computePlaneNormals_rb(
  struct vec p1,
  struct vec p2,
  struct vec p1_attached,
  struct vec p2_attached,
  float r,
  float l1,
  float l2, float lambda_svm, struct vec Fd1, struct vec Fd2,
  struct vec* n1, struct vec* n2,
  struct osqp_warmstart_hyperplane_rb* warmstart_state,
  float* solve_time_svm) {

  // TODO: relative to payload to aid warm start!

  // run QP to find separating hyperplane
  OSQPWorkspace* workspace = &workspace_hyperplane_rb;

  osqp_prepare_update(workspace);

  struct vec Fd1t = vadd(p1_attached, Fd1);
  struct vec Fd2t = vadd(p2_attached, Fd2);

  // Update A
  workspace->data->A->x[0] = p1.x;
  workspace->data->A->x[1] = p1_attached.x;
  workspace->data->A->x[2] = -p2.x;
  workspace->data->A->x[3] = -p2_attached.x;
  workspace->data->A->x[4] = Fd1t.x;
  workspace->data->A->x[5] = -Fd2t.x;

  workspace->data->A->x[6] = p1.y;
  workspace->data->A->x[7] = p1_attached.y;
  workspace->data->A->x[8] = -p2.y;
  workspace->data->A->x[9] = -p2_attached.y;
  workspace->data->A->x[10] = Fd1t.y;
  workspace->data->A->x[11] = -Fd2t.y;

  workspace->data->A->x[12] = p1.z;
  workspace->data->A->x[13] = p1_attached.z;
  workspace->data->A->x[14] = -p2.z;
  workspace->data->A->x[15] = -p2_attached.z;
  workspace->data->A->x[16] = Fd1t.z;
  workspace->data->A->x[17] = -Fd2t.z;

  // print_csc_matrix(workspace->data->P, "P");
  // print_csc_matrix(workspace->data->A, "A");
  // print_vec(workspace->data->q, workspace->data->n, "q");
  // print_vec(workspace->data->u, workspace->data->n, "u");
  // print_vec(workspace->data->l, workspace->data->n, "l");

  osqp_finalize_update(workspace);

  c_float q_new[6] = {0, 0, 0, lambda_svm, lambda_svm, 0 };

  osqp_update_lin_cost(workspace, q_new);

  osqp_warm_start(workspace, warmstart_state->x, warmstart_state->y);

  #ifdef CRAZYFLIE_FW
    uint64_t timestamp_svm_start = usecTimestamp();
  # else
    clock_t timestamp_svm_start = clock();
  #endif
  osqp_solve(workspace);
  #ifdef CRAZYFLIE_FW
    eventTrigger_qpSolved_payload.svm += (usecTimestamp() - timestamp_svm_start) / 8;
  #else
    *solve_time_svm += ((float)(clock() - timestamp_svm_start)) / CLOCKS_PER_SEC * 1000.0f;
  #endif

  if (workspace->info->status_val == OSQP_SOLVED/* || workspace->info->status_val == OSQP_SOLVED_INACCURATE || workspace->info->status_val == OSQP_MAX_ITER_REACHED*/) {
    memcpy(warmstart_state->x, workspace->solution->x, sizeof(warmstart_state->x));
    memcpy(warmstart_state->y, workspace->solution->y, sizeof(warmstart_state->y));

    struct vec n = vloadf(workspace->solution->x);
    // the QP adds additional decision variables...
    float a = workspace->solution->x[5];

    // static int counter = 0;
    // ++counter;
    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("hp %f %f %f %f\n", (double)n.x, (double)n.y, (double)n.z, (double)a);

    // }

    // For UAV1, compute the resulting intersection plane
    struct vec center;
    float radius;
    plane_sphere_intersection(n, a, p1_attached, l1, &center, &radius);

    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("int %f %f %f %f\n", (double)center.x, (double)center.y, (double)center.z, (double)radius);

    // }

    if (radius >= 0) {
      // compute the intersection point with the highest z-value
      struct vec point_on_plane = project_point_on_plane(n, a, vadd(center, mkvec(0,0,1)));
      struct vec dir = vnormalize(vsub(point_on_plane, center));
      struct vec intersection_high_z = vadd(center, vscl(radius, dir));

      // the updated plane connects three points (where p1 == new relative coordinate system)
      struct vec point0 = p1_attached;
      struct vec point1 = vadd(p1_attached, vcross(n, mkvec(0,0,1)));
      struct vec point2 = intersection_high_z;

      struct vec n1_untilted = vcross(vsub(point0, point1), vsub(point0, point2));

      // now rotate the hyperplane to compute the two resulting hyperplane
      struct vec axis = vcross(n, mkvec(0,0,1));

      if (r > 2.0f * l1) {
        *n1 = n1_untilted;
      } else {
        float angle1 = 2 * asinf(r / (2.0f * l1));
        struct quat q1 = qaxisangle(axis, angle1);
        *n1 = vneg(qvrot(q1, n1_untilted));
      }
    } else {
      //printf("no int 1\n");
      // the plane is so far, that we don't need a safety hyperplane!
      *n1 = vzero();
    }

    // For UAV2, compute the resulting intersection plane
    plane_sphere_intersection(n, a, p2_attached, l2, &center, &radius);

    if (radius >= 0) {
      // compute the intersection point with the highest z-value
      struct vec point_on_plane = project_point_on_plane(n, a, vadd(center, mkvec(0,0,1)));
      struct vec dir = vnormalize(vsub(point_on_plane, center));
      struct vec intersection_high_z = vadd(center, vscl(radius, dir));

      // the updated plane connects three points (where p2 == new relative coordinate system)
      struct vec point0 = p2_attached;
      struct vec point1 = vadd(p2_attached, vcross(n, mkvec(0,0,1)));
      struct vec point2 = intersection_high_z;

      struct vec n2_untilted = vcross(vsub(point0, point1), vsub(point0, point2));

      // now rotate the hyperplane to compute the two resulting hyperplane
      struct vec axis = vcross(n, mkvec(0,0,1));

      if (r > 2.0f * l1) {
        *n2 = n2_untilted;
      } else {
        float angle1 = 2 * asinf(r / (2.0f * l1));
        struct quat q1 = qaxisangle(axis, -angle1);
        *n2 = qvrot(q1, n2_untilted);
      }
    } else {
      // printf("no int 2 %f %f %f %f\n", p2_attached.x, p2_attached.y, p2_attached.z, l2);
      // printf("no int 2b %f %f %f %f\n", n.x, n.y, n.z, a);

      // the plane is so far, that we don't need a safety hyperplane!
      *n2 = vzero();
    }

  } else {
  #ifdef CRAZYFLIE_FW
        DEBUG_PRINT("QPsvm: %s\n", workspace->info->status);
        DEBUG_PRINT("QPsvm: %s %d %f\n", workspace->info->status, workspace->info->iter, (double)workspace->info->obj_val);
        DEBUG_PRINT("p1 = %f, %f, %f\n",(double) p1.x, (double)p1.y, (double)p1.z);
        DEBUG_PRINT("p2 = %f, %f, %f\n", (double)p2.x, (double)p2.y, (double)p2.z);
        DEBUG_PRINT("p1a = %f, %f, %f\n", (double)p1_attached.x, (double)p1_attached.y, (double)p1_attached.z);
        DEBUG_PRINT("p2a = %f, %f, %f\n", (double)p2_attached.x, (double)p2_attached.y, (double)p2_attached.z);
        DEBUG_PRINT("Fd1t = %f, %f, %f\n", (double)Fd1t.x, (double)Fd1t.y, (double)Fd1t.z);
        DEBUG_PRINT("Fd2t = %f, %f, %f\n", (double)Fd2t.x, (double)Fd2t.y, (double)Fd2t.z);
        DEBUG_PRINT("lambda_svm = %f\n", (double)lambda_svm);
  #else
        printf("QPsvm: %s %d %f\n", workspace->info->status, workspace->info->iter, workspace->info->obj_val);
        printf("p1 = %f, %f, %f\n", p1.x, p1.y, p1.z);
        printf("p2 = %f, %f, %f\n", p2.x, p2.y, p2.z);
        printf("p1a = %f, %f, %f\n", p1_attached.x, p1_attached.y, p1_attached.z);
        printf("p2a = %f, %f, %f\n", p2_attached.x, p2_attached.y, p2_attached.z);
        printf("Fd1t = %f, %f, %f\n", Fd1t.x, Fd1t.y, Fd1t.z);
        printf("Fd2t = %f, %f, %f\n", Fd2t.x, Fd2t.y, Fd2t.z);
        printf("lambda_svm = %f\n", lambda_svm);
  #endif

    *n1 = vzero();
    *n2 = vzero();
  }
}

static bool compute_Fd_pair_qp(struct quat payload_quat, struct vec attPoint1, struct vec attPoint2, struct vec F_d, struct vec M_d, struct vec* F_d1, struct vec* F_d2, struct osqp_warmstart_compute_Fd_pair* warmstart_state, float* solve_time_Fd)
{
  // printf("\nFd: %f %f %f\n", F_d.x, F_d.y, F_d.z);
  // printf("Md: %f %f %f\n", M_d.x, M_d.y, M_d.z);
  // printf("q: %f %f %f %f\n", payload_quat.x, payload_quat.y, payload_quat.z, payload_quat.w);
  // printf("ap1: %f %f %f\n", attPoint1.x, attPoint1.y, attPoint1.z);
  // printf("ap2: %f %f %f\n", attPoint2.x, attPoint2.y, attPoint2.z);

  struct mat33 R0t = mtranspose(quat2rotmat(payload_quat));

  struct mat33 attPR0t = mmul(mcrossmat(attPoint1), R0t);
  struct mat33 attP2R0t = mmul(mcrossmat(attPoint2), R0t);

  OSQPWorkspace* workspace = &workspace_compute_Fd_pair;

  osqp_prepare_update(workspace);

  // Update A
  c_float* x = workspace->data->A->x;
  x[ 0] = attPR0t.m[0][0];
  x[ 1] = attPR0t.m[1][0];
  x[ 2] = attPR0t.m[2][0];

  x[ 4] = attPR0t.m[0][1];
  x[ 5] = attPR0t.m[1][1];
  x[ 6] = attPR0t.m[2][1];

  x[ 8] = attPR0t.m[0][2];
  x[ 9] = attPR0t.m[1][2];
  x[10] = attPR0t.m[2][2];

  x[12] = attP2R0t.m[0][0];
  x[13] = attP2R0t.m[1][0];
  x[14] = attP2R0t.m[2][0];

  x[16] = attP2R0t.m[0][1];
  x[17] = attP2R0t.m[1][1];
  x[18] = attP2R0t.m[2][1];

  x[20] = attP2R0t.m[0][2];
  x[21] = attP2R0t.m[1][2];
  x[22] = attP2R0t.m[2][2];


  osqp_finalize_update(workspace);

  // update q, l, and u (after finalize update!)
  c_float l_new[6] =  {M_d.x, M_d.y, M_d.z, F_d.x, F_d.y, F_d.z};
  c_float u_new[6] =  {M_d.x, M_d.y, M_d.z, F_d.x, F_d.y, F_d.z};

  osqp_update_lower_bound(workspace, l_new);
  osqp_update_upper_bound(workspace, u_new);

  osqp_warm_start(workspace, warmstart_state->x, warmstart_state->y);

  #ifdef CRAZYFLIE_FW
    uint64_t timestamp_Fd_start = usecTimestamp();
  # else
    clock_t timestamp_Fd_start = clock();
  #endif
  osqp_solve(workspace);
  #ifdef CRAZYFLIE_FW
    eventTrigger_qpSolved_payload.Fd += (usecTimestamp() - timestamp_Fd_start) / 8;
  #else
    *solve_time_Fd += ((float)(clock() - timestamp_Fd_start)) / CLOCKS_PER_SEC * 1000.0f;
  #endif

  if (workspace->info->status_val == OSQP_SOLVED) {
    memcpy(warmstart_state->x, workspace->solution->x, sizeof(warmstart_state->x));
    memcpy(warmstart_state->y, workspace->solution->y, sizeof(warmstart_state->y));

    F_d1->x = (workspace)->solution->x[0];
    F_d1->y = (workspace)->solution->x[1];
    F_d1->z = (workspace)->solution->x[2];
    F_d2->x =  (workspace)->solution->x[3];
    F_d2->y =  (workspace)->solution->x[4];
    F_d2->z =  (workspace)->solution->x[5];

    // printf("Fd1: %f %f %f\n", F_d1->x, F_d1->y, F_d1->z);
    // printf("Fd2: %f %f %f\n", F_d2->x, F_d2->y, F_d2->z);



    return true;
  } else {
  #ifdef CRAZYFLIE_FW
        DEBUG_PRINT("QPp: %s\n", workspace->info->status);
  #else
        printf("QPp: %s\n", workspace->info->status);
  #endif
  }
  return false;
}


static controllerLeePayload_t g_self = {
  .mass = 0.034,
  .mp   = 0.01,
  // Inertia matrix (diagonal matrix), see
  // System Identification of the Crazyflie 2.0 Nano Quadrocopter
  // BA theses, Julian Foerster, ETHZ
  // https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  // Position PID
  .Kpos_P = {18, 18, 18},
  .Kpos_P_limit = 100,
  .Kpos_D = {15, 15, 15},
  .Kpos_D_limit = 100,
  .Kpos_I ={10, 10, 10},
  .Kpos_I_limit = 0,

  // Payload attitude gains

  .Kprot_P = {0.01, 0.01, 0.01},
  .Kprot_P_limit = 100,
  .Kprot_D = {0.005, 0.005, 0.005},
  .Kprot_D_limit = 100,
  .Kprot_I = {0.0, 0.0, 0.0},
  .Kprot_I_limit = 100,

  // Cables PD
  .K_q = {25, 25, 25},
  .K_q_limit = 100,
  .K_w = {24, 24, 24},
  .K_w_limit = 100,
  .K_q_I = {0, 0, 0},

  //Attitude PID 
  .KR = {0.008, 0.008, 0.01},
  .KR_limit = 100,
  .Komega = {0.0013, 0.0013, 0.002},
  .Komega_limit = 100,
  .KI = {0.02, 0.02, 0.05},
  // desired orientation and omega for payload
  .qp_des = {0, 0, 0, 1},
  .wp_des = {0, 0, 0},

  .en_qdidot = 0,
  .en_accrb = 1,
  .formation_control = 0,

  .radius = 0.15,

  .lambdaa = 0.0,
  .lambda_svm = 0.0,
  .gen_hp = 0,

  // automatically compute by default
  .attachement_points[0].l = -1,
  .attachement_points[1].l = -1,
  .attachement_points[2].l = -1,

  .Pinvs[0].Pinv.m = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}},
  .Pinvs[1].Pinv.m = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}},
  .Pinvs[2].Pinv.m = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}},

  // INDI
  .indi = 0,
  .est_acc = 0,
  .use_nn = 0,
};


static bool rpm_deck_available;
static logVarId_t logVarRpm1;
static logVarId_t logVarRpm2;
static logVarId_t logVarRpm3;
static logVarId_t logVarRpm4;

static Butterworth2LowPass filter_acc_rpm[3];
static Butterworth2LowPass filter_acc_imu[3];
static Butterworth2LowPass filter_tau_rpm[3];
static Butterworth2LowPass filter_angular_acc[3];

extern float kappa_f[4];

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

// static inline void update_butterworth_2_low_pass_vec(Butterworth2LowPass filter[3], struct vec value) {
//   update_butterworth_2_low_pass(&filter[0], value.x);
//   update_butterworth_2_low_pass(&filter[1], value.y);
//   update_butterworth_2_low_pass(&filter[2], value.z);
// }

// static inline struct vec get_butterworth_2_low_pass_vec(Butterworth2LowPass filter[3]) {
//   float x = get_butterworth_2_low_pass(&filter[0]);
//   float y = get_butterworth_2_low_pass(&filter[1]);
//   float z = get_butterworth_2_low_pass(&filter[2]);
//   return mkvec(x, y, z);
// }

// static inline struct vec vclampscl(struct vec value, float min, float max) {
//   return mkvec(
//     clamp(value.x, min, max),
//     clamp(value.y, min, max),
//     clamp(value.z, min, max));
// }

static void runQP(const struct QPInput *input, struct QPOutput* output)
{
#ifdef CRAZYFLIE_FW
  eventTrigger_qpSolved_payload.Fd = 0;
  eventTrigger_qpSolved_payload.svm = 0;
  eventTrigger_qpSolved_payload.mu = 0;
  uint64_t timestamp_total_start = usecTimestamp();
# else
  input->self->solve_time_Fd = 0;
  input->self->solve_time_svm = 0;
  input->self->solve_time_mu = 0;
  clock_t timestamp_total_start = clock();
#endif

  struct vec F_d = input->F_d;
  struct vec plStPos = input->plStPos;
  float radius = input->self->radius;
  output->timestamp = input->timestamp;
  output->success = false;

  float ls[MAX_TEAM_SIZE];

  for (uint8_t i=0; i < input->num_uavs; ++i)
  {
    if (input->team_state[i].l <= 0) {
      ls[i] = vmag(vsub(plStPos, input->team_state[i].pos));
    } else {
      ls[i] = input->team_state[i].l;
    }
  }
  uint8_t num_uavs = input->num_uavs;
  uint8_t num_hps = num_uavs*(num_uavs - 1);

  bool is_rigid_body = !isnanf(input->plStquat.w);

  if(!is_rigid_body) {
    // N uavs point mass case
    for (uint8_t i=0; i < input->num_uavs; ++i) {
      for (uint8_t j=i+1; j < input->num_uavs; ++j) {
        // 0,1; 0, 2; 1,2; 
        uint8_t n_idx1 = i * (input->num_uavs-1) + j - 1; // 0, 1, 3
        uint8_t n_idx2 = i + j * (input->num_uavs-1); // 2, 4, 5 
        computePlaneNormals(input->team_state[i].pos, input->team_state[j].pos, plStPos, radius, ls[i], ls[j], input->self->lambda_svm, F_d, &input->self->n[n_idx1], &input->self->n[n_idx2], &input->self->osqp_warmstart_hyperplanes[0], &input->self->solve_time_svm); 
      }
    }

    OSQPWorkspace* workspace = 0;
    if (input->num_uavs == 2) {
      workspace = &workspace_2uav_2hp;
    } 
    else if (input->num_uavs == 3) {
      workspace = &workspace_3uav_2hp;
    } else if (input->num_uavs == 4) {
      workspace = &workspace_4uav_5hp;
    } 
    // else if (input->num_uavs == 5) {
    //   workspace = &workspace_5uav_9hp;
    // }  else if (input->num_uavs == 6) {
    //   workspace = &workspace_6uav_14hp;
    // } else if (input->num_uavs == 7) {
    //   workspace = &workspace_7uav_42hp;
    // } else if (input->num_uavs == 8){
    //   workspace = &workspace_8uav_56hp;
    // } else if (input->num_uavs == 9) {
    //   workspace = &workspace_9uav_72hp;
    // } else if (input->num_uavs == 10) {
    //   workspace = &workspace_10uav_90hp;
    // }
    // workspace structure
    workspace->settings->warm_start = 1;
    osqp_prepare_update(workspace);
    c_float* x = workspace->data->A->x;
    // printf("lenx: %d\n",workspace->data->A->nzmax);
    //  The new code
    uint8_t jump = 2*num_uavs + 2;
    uint8_t jump_counter = 1;
    uint16_t j = 1;

    for (uint8_t i = 0; i < num_hps; ++i) {
      x[j] = input->self->n[i].x;
      x[j+num_uavs] = input->self->n[i].y;
      x[j+2*num_uavs] = input->self->n[i].z;
      // printf("j: %d %d %d, jump: %d, jump_counter: %d\n", j, j+num_uavs, j+2*num_uavs, jump, jump_counter);
      if (jump_counter < num_uavs-1) {
        j+=1;
        jump_counter+=1;
      } else {
        j += jump;
        jump_counter = 1;
      }
    }
    osqp_finalize_update(workspace);
   
  
    c_float l_new[3 + num_hps];
    c_float u_new[3 + num_hps];
    c_float q_new[3*num_uavs];

    l_new[0] = F_d.x;
    l_new[1] = F_d.y;
    l_new[2] = F_d.z;
    u_new[0] = F_d.x; 
    u_new[1] = F_d.y;
    u_new[2] = F_d.z; 
    for (uint8_t i = 3; i < 3 + num_hps; ++i) {
        l_new[i] = -INFINITY;
        u_new[i] = 0;
    }
    
    const float factor = - 2.0f * input->self->lambdaa / (1.0f + 2.0f * input->self->lambdaa);
    
    for (uint8_t i = 0; i < num_uavs; ++i) {
      if (input->self->formation_control == 0) {
        q_new[3*i]     = 0.0f;
        q_new[3*i + 1] = 0.0f;
        q_new[3*i + 2] = 0.0f;
      } else if (input->self->formation_control == 1) {
        q_new[3*i]     = factor * input->self->desVirt_prev[i].x;
        q_new[3*i + 1] = factor * input->self->desVirt_prev[i].y;
        q_new[3*i + 2] = factor * input->self->desVirt_prev[i].z;
      } else {
        // DEBUG_PRINT("mu_ref: %f %f %f\n", (double) input->team_state[i].mu_ref.x, (double) input->team_state[i].mu_ref.y, (double) input->team_state[i].mu_ref.z);
        q_new[3*i]     = factor * input->team_state[i].mu_ref.x;
        q_new[3*i + 1] = factor * input->team_state[i].mu_ref.y; 
        q_new[3*i + 2] = factor * input->team_state[i].mu_ref.z;
      }
    }
    // printf("q_new: %f %f %f %f %f %f\n",q_new[0], q_new[1], q_new[2], q_new[3], q_new[4],q_new[5]);
    osqp_update_lin_cost(workspace, q_new);
    osqp_update_lower_bound(workspace, l_new);
    osqp_update_upper_bound(workspace, u_new);

    #ifdef CRAZYFLIE_FW
      uint64_t timestamp_mu_start = usecTimestamp();
    #else
      clock_t timestamp_mu_start = clock();
    #endif
    osqp_solve(workspace);
    #ifdef CRAZYFLIE_FW
      eventTrigger_qpSolved_payload.mu += (usecTimestamp() - timestamp_mu_start) / 8;
    #else
      input->self->solve_time_mu += ((float)(clock() - timestamp_mu_start)) / CLOCKS_PER_SEC * 1000.0f;
    #endif

    if (workspace->info->status_val == OSQP_SOLVED) {
      for (uint8_t i = 0; i < num_uavs; ++i) {

      input->self->desVirt[i].x = (workspace)->solution->x[3*i];
      input->self->desVirt[i].y = (workspace)->solution->x[3*i + 1];
      input->self->desVirt[i].z = (workspace)->solution->x[3*i + 2];

      input->self->desVirt_prev[i].x =  (workspace)->solution->x[3*i];
      input->self->desVirt_prev[i].y =  (workspace)->solution->x[3*i + 1];
      input->self->desVirt_prev[i].z =  (workspace)->solution->x[3*i + 2];
      }

      output->success = true;
    } else {
    #ifdef CRAZYFLIE_FW
          DEBUG_PRINT("QP: %s\n", workspace->info->status);
    #else
          printf("QP: %s\n", workspace->info->status);
    #endif
    }
    output->desVirtInp.x = input->self->desVirt[0].x; 
    output->desVirtInp.y = input->self->desVirt[0].y; 
    output->desVirtInp.z = input->self->desVirt[0].z; 
    #ifdef CRAZYFLIE_FW
      // for (uint8_t i = 0; i < input->num_uavs; ++i) { 
      //   DEBUG_PRINT("mu_des: %d = [%f %f %f] \n", i, (double) input->self->desVirt[i].x, (double) input->self->desVirt[i].y, (double) input->self->desVirt[i].z);
      // }

      // for (uint8_t i = 0; i < input->num_uavs; ++i) { 
        // DEBUG_PRINT("mu_des_i: %d = [%f %f %f] \n", i, (double) input->self->desVirt[i].x, (double) input->self->desVirt[i].y, (double) input->self->desVirt[i].z);
        // DEBUG_PRINT("Fd: %d = [%f %f %f] \n", i, (double) input->F_d.x, (double) input->F_d.y, (double) input->F_d.z);
      // }
    #endif
  } else {
    // rigid general case
    struct vec M_d = input->M_d;
    struct vec Fd1 = mkvec(0,0,0);
    struct vec Fd2 = mkvec(0,0,0);

    for (uint8_t i=0; i < input->num_uavs; ++i) {
      for (uint8_t j=i+1; j < input->num_uavs; ++j) {
        // 0,1; 0, 2; 1,2; 
        uint8_t n_idx1 = i * (input->num_uavs-1) + j - 1; // 0, 1, 3
        uint8_t n_idx2 = i + j * (input->num_uavs-1); // 2, 4, 5 

        compute_Fd_pair_qp(input->plStquat, input->team_state[i].attPoint, input->team_state[j].attPoint, F_d, M_d, &Fd1, &Fd2, &input->self->osqp_warmstart_compute_Fd_pairs[0], &input->self->solve_time_Fd);

        struct vec plSt_att_i = vadd(plStPos, qvrot(input->plStquat, input->team_state[i].attPoint));
        struct vec plSt_att_j = vadd(plStPos, qvrot(input->plStquat, input->team_state[j].attPoint));
        
        computePlaneNormals_rb(input->team_state[i].pos, input->team_state[j].pos, plSt_att_i, plSt_att_j, radius, ls[i], ls[j], input->self->lambda_svm, Fd1, Fd2, &input->self->n[n_idx1], &input->self->n[n_idx2], &input->self->osqp_warmstart_hyperplane_rbs[0], &input->self->solve_time_svm);
      }
    }

    // new code for rigid case
    struct mat33 R0t = mtranspose(quat2rotmat(input->plStquat));

    OSQPWorkspace* workspace = 0;
    // if (input->num_uavs == 2) {
    //   workspace = &workspace_2uav_1hp_rod;
    // } else if (input->num_uavs == 3) {
    //   workspace = &workspace_3uav_6hp_rig;
    // } 
    // else if (input->num_uavs == 4) {
    //   workspace = &workspace_4uav_12hp_rig;
    // } 
    // else if (input->num_uavs == 5) {
    //   workspace = &workspace_5uav_20hp_rig;
    // } else if (input->num_uavs == 6) {
    //   workspace = &workspace_6uav_30hp_rig;
    // } else if (input->num_uavs == 7) {
    //   workspace = &workspace_7uav_42hp_rig;
    // } else if (input->num_uavs == 8) {
    //   workspace = &workspace_8uav_56hp_rig;
    // } else if (input->num_uavs == 9) {
    //   workspace = &workspace_9uav_72hp_rig;
    // } else if (input->num_uavs == 10) {
    //   workspace = &workspace_10uav_90hp_rig;
    // }
    
    // workspace structure
    workspace->settings->warm_start = 1;

    osqp_prepare_update(workspace);
    
    c_float* x = workspace->data->A->x;
    uint32_t len_x = workspace->data->A->nzmax;
    
    // add R0t and attPR0t in A
    uint32_t step = 5 + num_uavs;
    uint8_t rot_counter = 0;
    uint8_t attP_idx = 0;
    
    for (uint32_t i = 0; i < len_x; i+=step) {
        struct vec R0t_vec = mcolumn(R0t, rot_counter);
        struct mat33 attPR0t = mmul(mcrossmat(input->team_state[attP_idx].attPoint), R0t);
        struct vec attPR0t_vec = mcolumn(attPR0t, rot_counter);
    
        x[i]   = R0t_vec.x;
        x[i+1] = R0t_vec.y;
        x[i+2] = R0t_vec.z;
        x[i+3] = attPR0t_vec.x;
        x[i+4] = attPR0t_vec.y;
        x[i+5] = attPR0t_vec.z;
        if (rot_counter == 2){
          rot_counter = 0;
          attP_idx++;
        } else {
          rot_counter++;
        }
    }

    // add hyperplanes n in A
    uint32_t jump = 2*num_uavs + 17;
    uint8_t jump_counter = 1;
    uint32_t j = 6;
    for (uint32_t i = 0; i < num_hps; ++i) {
      x[j] = input->self->n[i].x;
      x[j+num_uavs+5] = input->self->n[i].y;
      x[j+2*(num_uavs + 5)] = input->self->n[i].z;

      if (jump_counter < num_uavs - 1) {
        j+=1;
        jump_counter+=1;
      } else {
        j += jump;
        jump_counter = 1;
      }
    }
    osqp_finalize_update(workspace);

    c_float l_new[6 + num_hps];
    c_float u_new[6 + num_hps];
    c_float q_new[3*num_uavs];

    l_new[0] = F_d.x;
    l_new[1] = F_d.y;
    l_new[2] = F_d.z;
    l_new[3] = M_d.x;
    l_new[4] = M_d.y;
    l_new[5] = M_d.z;

    u_new[0] = F_d.x; 
    u_new[1] = F_d.y;
    u_new[2] = F_d.z; 
    u_new[3] = M_d.x; 
    u_new[4] = M_d.y; 
    u_new[5] = M_d.z; 

    for (uint16_t i = 6; i < 6 + num_hps; ++i) {
        l_new[i] = -INFINITY;
        u_new[i] = 0;
    }

    const float factor = - 2.0f * input->self->lambdaa / (1.0f + 2.0f * input->self->lambdaa);
    
    for (uint16_t i = 0; i < num_uavs; ++i) {
      if (input->self->formation_control == 0) {
        q_new[3*i]     = 0.0f;
        q_new[3*i + 1] = 0.0f;
        q_new[3*i + 2] = 0.0f;
      } else if (input->self->formation_control == 1) {
        q_new[3*i]     = factor * input->self->desVirt_prev[i].x;
        q_new[3*i + 1] = factor * input->self->desVirt_prev[i].y;
        q_new[3*i + 2] = factor * input->self->desVirt_prev[i].z;
      } else {
        struct vec mu_ref  = input->team_state[i].mu_ref; 
        q_new[3*i]     = factor * mu_ref.x;
        q_new[3*i + 1] = factor * mu_ref.y; 
        q_new[3*i + 2] = factor * mu_ref.z;
      }
    }
    osqp_update_lin_cost(workspace, q_new);
    osqp_update_lower_bound(workspace, l_new);
    osqp_update_upper_bound(workspace, u_new);

    #ifdef CRAZYFLIE_FW
      uint64_t timestamp_mu_start = usecTimestamp();
    #else
      clock_t timestamp_mu_start = clock();
    #endif

    osqp_solve(workspace);
    
    #ifdef CRAZYFLIE_FW
      eventTrigger_qpSolved_payload.mu += (usecTimestamp() - timestamp_mu_start) / 8;
    #else
      input->self->solve_time_mu += ((float)(clock() - timestamp_mu_start)) / CLOCKS_PER_SEC * 1000.0f;
    #endif

    if (workspace->info->status_val == OSQP_SOLVED) {
      for (uint8_t i = 0; i < num_uavs; ++i) {

      input->self->desVirt[i].x = (workspace)->solution->x[3*i];
      input->self->desVirt[i].y = (workspace)->solution->x[3*i + 1];
      input->self->desVirt[i].z = (workspace)->solution->x[3*i + 2];

      input->self->desVirt_prev[i].x =  (workspace)->solution->x[3*i];
      input->self->desVirt_prev[i].y =  (workspace)->solution->x[3*i + 1];
      input->self->desVirt_prev[i].z =  (workspace)->solution->x[3*i + 2];
      }
      output->success = true;
    } else {
    #ifdef CRAZYFLIE_FW
          DEBUG_PRINT("QP: %s\n", workspace->info->status);
    #else
          printf("QP: %s\n", workspace->info->status);
    #endif
    }
    output->desVirtInp.x = input->self->desVirt[0].x; 
    output->desVirtInp.y = input->self->desVirt[0].y; 
    output->desVirtInp.z = input->self->desVirt[0].z; 
  } 

///
  
#ifdef CRAZYFLIE_FW
  eventTrigger_qpSolved_payload.total = (usecTimestamp() - timestamp_total_start) / 8;
  eventTrigger(&eventTrigger_qpSolved);
# else
  input->self->solve_time_total = ((float) (clock() - timestamp_total_start)) / CLOCKS_PER_SEC * 1000.0f;
#endif

}

static inline struct vec computeUnitVec(float az, float el)
{
  // cos(az)*np.cos(el)
  return mkvec(cosf(az)*cosf(el), sinf(az)*cosf(el), sin(el));
}

static void computeDesiredVirtualInput(controllerLeePayload_t* self, const state_t *state, const setpoint_t* setpoint, struct vec F_d, struct vec M_d, uint32_t tick_in, struct vec* result, uint32_t* tick_out)
{
  struct QPInput qpinput;
  struct QPOutput qpoutput;
  // dirty hack: need to rewrite it
  memset(&qpinput, 0, sizeof(qpinput));
  // for (uint8_t i = 0; i < state->num_uavs; ++i) {
  //   qpinput.team_state[i].mu_ref = vzero();
  // }
  // push the latest change to the QP
  qpinput.F_d = F_d;
  qpinput.M_d = M_d;
  qpinput.plStPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
  qpinput.plStquat = mkquat(state->payload_quat.x, state->payload_quat.y, state->payload_quat.z, state->payload_quat.w);
  // if (state->num_neighbors == 1) {
  //   struct vec rpy = quat2rpy(qpinput.plStquat);
  //   rpy.y = 0;
  //   qpinput.plStquat = rpy2quat(rpy);
  // }
  qpinput.num_uavs = state->num_uavs;
  for (uint8_t i = 0; i < state->num_uavs; ++i) {
    qpinput.team_state[i].id = state->team_state[i].id;
    qpinput.team_state[i].pos = mkvec(state->team_state[i].pos.x, state->team_state[i].pos.y, state->team_state[i].pos.z);
    //qpinput.team_state[i].mu_ref = setpoint->cablevectors[i].mu_ref;
    //qpinput.team_state[i].attPoint = self->attachement_points[i].point;

    for (uint8_t j = 0; j < state->num_uavs; ++j) {
      if (self->attachement_points[j].id == state->team_state[i].id) {
        qpinput.team_state[i].attPoint = self->attachement_points[j].point;
        qpinput.team_state[i].l = self->attachement_points[j].l;
        break;
      }
    }
    for (uint8_t j = 0; j < state->num_uavs; ++j) {
      if (setpoint->cablevectors[j].id == state->team_state[i].id) {
        qpinput.team_state[i].mu_ref = setpoint->cablevectors[j].mu_ref;
        break;
      }
    }
    self->mu_ref = qpinput.team_state[0].mu_ref;
  }

  qpinput.self = self;
  qpinput.timestamp = tick_in;

#ifdef CRAZYFLIE_FW
  xQueueOverwrite(queueQPInput, &qpinput);
  // get the latest result from the async computation, do not wait to not block the main loop
  BaseType_t qr = xQueuePeek(queueQPOutput, &qpoutput, 0);

    // static int counter = 0;
    // ++counter;
    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("db %d %d\n", (int)qr, qpoutput.success);
    // }

  // also mark as not successful, if queue was empty for some reason
  qpoutput.success &= (qr == pdTRUE);
#else
  // solve the QP
  runQP(&qpinput, &qpoutput);
#endif
  if (qpoutput.success) {
    *tick_out = qpoutput.timestamp;
    *result = qpoutput.desVirtInp;
  }
}

#ifdef CRAZYFLIE_FW

void controllerLeePayloadQPTask(void * prm)
{
  struct QPInput qpinput;
  struct QPOutput qpoutput;

  while(1) {
    // wait until we get the next request
    xQueueReceive(queueQPInput, &qpinput, portMAX_DELAY);

    // solve the QP
    uint64_t start = usecTimestamp();
    runQP(&qpinput, &qpoutput);
    uint64_t end = usecTimestamp();
    qp_runtime_us = end - start;
    // store the result
    xQueueOverwrite(queueQPOutput, &qpoutput);
  }
}
#endif


static inline void update_butterworth_2_low_pass_vec(Butterworth2LowPass filter[3], struct vec value)
{
	update_butterworth_2_low_pass(&filter[0], value.x);
	update_butterworth_2_low_pass(&filter[1], value.y);
	update_butterworth_2_low_pass(&filter[2], value.z);
}

static inline struct vec get_butterworth_2_low_pass_vec(Butterworth2LowPass filter[3]) {
  float x = get_butterworth_2_low_pass(&filter[0]);
  float y = get_butterworth_2_low_pass(&filter[1]);
  float z = get_butterworth_2_low_pass(&filter[2]);
  return mkvec(x, y, z);
}

static Butterworth2LowPass filter_payload_vel[3];
static Butterworth2LowPass filter_payload_acc[3];

void controllerLeePayloadReset(controllerLeePayload_t* self)
{
  // DEBUG_PRINT("RST\n");
  self->i_error_pos = vzero();
  self->i_error_att = vzero();
  self->i_error_q = vzero();
  self->i_error_pl_att = vzero();
  self->qi_prev = mkvec(0,0,-1);
  self->qidot_prev = vzero();
  self->acc_prev   = vzero();
  self->payload_vel_prev = vzero();
  self->qdi_prev = vzero();
  self->desVirtInp = vzero();
  self->prev_q_des = mkquat(0, 0, 0, 1);
  
  self->delta_bar_x0 = vzero();
  self->delta_bar_R0 = vzero();
  self->delta_bar_xi = vzero();

  #ifdef CRAZYFLIE_FW
  xQueueReset(queueQPInput);
  xQueueReset(queueQPOutput);
  #endif
}

void controllerLeePayloadInit(controllerLeePayload_t* self)
{
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;
  DEBUG_PRINT("R3\n");
  controllerLeePayloadReset(self);
  time_start = usecTimestamp();

  for (int8_t i = 0; i < 3; i++) {
    const float cutoff = 70; // Hz
    const float cutoff_acc = 30; // Hz
    init_butterworth_2_low_pass(&filter_payload_vel[i], 1 / (2 * M_PI_F * cutoff), 1.0 / ATTITUDE_RATE, 0.0f);
    init_butterworth_2_low_pass(&filter_payload_acc[i], 1 / (2 * M_PI_F * cutoff_acc), 1.0 / ATTITUDE_RATE, 0.0f);
  }

  paramVarId_t idDeckBcRpm = paramGetVarId("deck", "bcRpm");
  logVarRpm1 = logGetVarId("rpm", "m1");
  logVarRpm2 = logGetVarId("rpm", "m2");
  logVarRpm3 = logGetVarId("rpm", "m3");
  logVarRpm4 = logGetVarId("rpm", "m4");

  rpm_deck_available = (paramGetUint(idDeckBcRpm) == 1);

	for (int8_t i = 0; i < 3; i++) {
    const float cutoff = 30; // Hz
		init_butterworth_2_low_pass(&filter_acc_rpm[i], 1 / (2 * M_PI_F * cutoff), 1.0 / ATTITUDE_RATE, 0.0f);
		init_butterworth_2_low_pass(&filter_acc_imu[i], 1 / (2 * M_PI_F * cutoff), 1.0 / ATTITUDE_RATE, 0.0f);

		init_butterworth_2_low_pass(&filter_tau_rpm[i], 1 / (2 * M_PI_F * cutoff), 1.0 / ATTITUDE_RATE, 0.0f);
		init_butterworth_2_low_pass(&filter_angular_acc[i], 1 / (2 * M_PI_F * cutoff), 1.0 / ATTITUDE_RATE, 0.0f);
	}

  self->timestamp_prev = usecTimestamp();
  self->omega_prev = vzero();
  
  // Acceleration estimation
  self->payload_vel_prev = vzero();
  self->timestamp_payload_prev = usecTimestamp();
}

bool controllerLeePayloadTest(controllerLeePayload_t* self)
{
  return true;
}

void controllerLeePayload(controllerLeePayload_t* self, control_t *control, const setpoint_t *setpoint,
                                                        const sensorData_t *sensors,
                                                        const state_t *state,
                                                        const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  // uint64_t startTime = usecTimestamp();

  float dt = (float)(1.0f/ATTITUDE_RATE);

  // INDI
  float t1 = 0.0f, t2 = 0.0f, t3 = 0.0f, t4 = 0.0f;
  if (self->indi && rpm_deck_available) {
    // compute expected acceleration based on rpm measurements (world frame)
    uint16_t rpm1 = logGetUint(logVarRpm1);
    uint16_t rpm2 = logGetUint(logVarRpm2);
    uint16_t rpm3 = logGetUint(logVarRpm3);
    uint16_t rpm4 = logGetUint(logVarRpm4);

    t1 = kappa_f[0] * powf(rpm1, 2);
    t2 = kappa_f[1] * powf(rpm2, 2);
    t3 = kappa_f[2] * powf(rpm3, 2);
    t4 = kappa_f[3] * powf(rpm4, 2);

    // DEBUG
    if (tick % 500 == 0) {
      
      DEBUG_PRINT("INDI t %f %f %f %f\n", (double)t1, (double)t2, (double)t3, (double)t4);
    }
  }


  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {  
    
    struct vec plPos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec plVel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec plAcc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE);

    struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
    struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
    struct vec plStPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
    struct vec plStVel_unfiltered = mkvec(state->payload_vel.x, state->payload_vel.y, state->payload_vel.z);

    update_butterworth_2_low_pass_vec(filter_payload_vel, plStVel_unfiltered);

    struct vec plStVel = get_butterworth_2_low_pass_vec(filter_payload_vel);
    self->plVel_filtered = plStVel;
    // rotational states of the payload
    struct quat plquat = mkquat(state->payload_quat.x, state->payload_quat.y, state->payload_quat.z, state->payload_quat.w);
    struct vec plomega = mkvec(state->payload_omega.x, state->payload_omega.y, state->payload_omega.z);

    // errors
    struct vec plpos_e = vclampnorm(vsub(plPos_d, plStPos), self->Kpos_P_limit);
    struct vec plvel_e = vclampnorm(vsub(plVel_d, plStVel), self->Kpos_D_limit);
    self->i_error_pos = vclampnorm(vadd(self->i_error_pos, vscl(dt, plpos_e)), self->Kpos_I_limit);

    // static int counter = 0;
    // ++counter;
    // if (counter % 1000 == 0) {
    //   DEBUG_PRINT("desPos: %f %f %f\n", (double) plPos_d.x, (double) plPos_d.y, (double) plPos_d.z);
    //   DEBUG_PRINT("setpoint: %f %f %f\n", (double) setpoint->position.x, (double) setpoint->position.y, (double) setpoint->position.z);
    //   DEBUG_PRINT("PlStPos: %f %f %f\n", (double) plStPos.x, (double) plStPos.y, (double) plStPos.z);
    // }
    // Lee's integral error (30)
    self->delta_bar_x0 = vadd(self->delta_bar_x0, vscl(self->h_x0/self->mp*dt, vadd(plvel_e, vscl(self->c_x, plpos_e))));

    struct vec attPoint = mkvec(0, 0, 0);
    float l = -1;
    // find the attachment point for this UAV (the one, which doesn't have any neighbor associated with it)
    for (uint8_t i = 0; i < state->num_uavs; ++i) {
      if (self->attachement_points[i].id == state->team_state[0].id) {
        // this attachement point belongs to a neighbor

        attPoint = self->attachement_points[i].point;
        l = self->attachement_points[i].l;
        break;
      }
    }

    // find the qid_ref for this UAV
    self->qid_ref = vzero();
    for (uint8_t j = 0; j < state->num_uavs; ++j) {
      if (setpoint->cablevectors[j].id == state->team_state[0].id) {
        self->qid_ref = setpoint->cablevectors[j].qid_ref;
        break;
      }
    }

    // Set corresponding desired cable angles points
    // static int counter = 0;
    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("formation control: %d\n", self->formation_control);
    //   DEBUG_PRINT("cable nums %d\n", setpoint->num_cables);
    //   DEBUG_PRINT("num of neighbors: %d\n", state->num_neighbors);
    // }
    // counter++;

    // for (uint8_t i = 0; i < state->num_uavs; ++i) {
    //   for (uint8_t j = 0; j < state->num_uavs; ++j) {
    //     if (setpoint->cableAngles[j].id == state->team_state[i].id) {
    //       float az = setpoint->cableAngles[j].az;
    //       float el = setpoint->cableAngles[j].el;
    //       self->desiredCableUnitVec[i] = computeUnitVec(az, el);
    //       break;
    //     }
    //   }  
    // }
    // for (uint8_t i = 0; i < setpoint->num_cables; ++i) {
    //   // DEBUG_PRINT("num of neighbors: %d\n", state->num_neighbors);
    //   // DEBUG_PRINT("setpoint id: %d\n", setpoint->cableAngles[i].id);
    //   // DEBUG_PRINT("uav id 1: %d\n", state->neighbors[0].id);
    //   // DEBUG_PRINT("uav id 2: %d\n", state->neighbors[1].id);
      
    //   if (state->num_neighbors > 0 && setpoint->cableAngles[i].id == state->neighbors[0].id) {
    //     float az2 = setpoint->cableAngles[i].az;
    //     float el2 = setpoint->cableAngles[i].el;
    //     self->desiredCableUnitVec[1] = computeUnitVec(az2, el2);
    //   } else if (state->num_neighbors > 1 && setpoint->cableAngles[i].id == state->neighbors[1].id) {
    //     float az3 = setpoint->cableAngles[i].az;
    //     float el3 = setpoint->cableAngles[i].el;        
    //     self->desiredCableUnitVec[2] = computeUnitVec(az3, el3);
        
    //   } else {
    //     float az = setpoint->cableAngles[i].az;
    //     float el = setpoint->cableAngles[i].el;
    //     self->desiredCableUnitVec[0] = computeUnitVec(az, el);
    //   }
    // }
    // DEBUG_PRINT("qi = [%f, %f, %f]\n", (double) self->desiredCableUnitVec.x, (double) self->desiredCableUnitVec.y, (double) self->desiredCableUnitVec.z);
    // DEBUG_PRINT("qi2 = [%f, %f, %f]\n", (double) self->desiredCableUnitVec2.x, (double) self->desiredCableUnitVec2.y, (double) self->desiredCableUnitVec2.z);
    if (!isnanf(plquat.w)) {
      // If the payload is a rigid body then the the attachment point should be added to PlStPos
      plStPos = vadd(plStPos, qvrot(plquat, attPoint));
    }

    // automatically compute cable length, if desired
    if (l <= 0) {
      l = vmag(vsub(plStPos, statePos));
    }

    // payload orientation errors
    // payload quat to R 
    struct mat33 Rp = quat2rotmat(plquat);
    // define desired payload Rp_des = eye(3) (i.e., qp_des = [0,0,0,1] (x,y,z,w))

    // Address inconsistency in firmware where we need to compute our own desired yaw angle
    // Rate-controlled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
      float desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
      self->qp_des = rpy2quat(mkvec(self->roll_des,self->pitch_des,desiredYaw));
    } else if (setpoint->mode.yaw == modeAbs) {
      float desiredYaw = radians(setpoint->attitude.yaw);
      self->qp_des = rpy2quat(mkvec(self->roll_des,self->pitch_des,desiredYaw));
    } else if (setpoint->mode.quat == modeAbs) {
      self->qp_des = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    }
  
    struct mat33 Rp_des = quat2rotmat(self->qp_des); 
    // define orientation error     
    // eRp =  msub(mmul(mtranspose(self->R_des), self->R), mmul(mtranspose(self->R), self->R_des));
    struct mat33 eRMp =  msub(mmul(mtranspose(Rp_des), Rp), mmul(mtranspose(Rp), Rp_des));
    struct vec eRp = vscl(0.5f, mkvec(eRMp.m[2][1], eRMp.m[0][2], eRMp.m[1][0]));
    
    self->i_error_pl_att = vclampnorm(vadd(self->i_error_pl_att, vscl(dt, eRp)), self->Kprot_I_limit);

    self->wp_des = mkvec(radians(setpoint->attitudeRate.roll), radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw));
    self->omega_pr = mvmul(mmul(mtranspose(Rp), Rp_des), self->wp_des);
    struct vec omega_perror = vsub(plomega, self->omega_pr);

    // Lees integral error (31)
    self->delta_bar_R0 = vadd(self->delta_bar_R0, vscl(dt*self->h_R0, vadd(omega_perror, vscl(self->c_R, eRp))));

    self->plp_error = plpos_e;
    self->plv_error = plvel_e;

    // Lee (20)
    // Note that the last component is not included here, since it would require
    // computing delta_bar_xi for all i, on each robot
    self->F_d = vsub(
        vscl(self->mp ,vadd4(
          plAcc_d,
          veltmul(self->Kpos_P, plpos_e),
          veltmul(self->Kpos_D, plvel_e),
          veltmul(self->Kpos_I, self->i_error_pos))), // not in the original formulation
        self->delta_bar_x0
      );

    // Lee (21)
    // Note that the part with omega_0_d and omega_0_d_dot are not included, as they are zero in our case

    // Note that the last component is not included here, since it would require
    // computing delta_bar_xi for all i, on each robot
    self->M_d = vadd4(
      vneg(veltmul(self->Kprot_P, vclampnorm(eRp, self->Kprot_P_limit))),
      vneg(veltmul(self->Kprot_D, vclampnorm(omega_perror, self->Kprot_D_limit))),
      vneg(veltmul(self->Kprot_I, self->i_error_pl_att)), // not in the original formulation
      vneg(self->delta_bar_R0)
    );


    if (state->num_uavs > 1) {
      computeDesiredVirtualInput(self, state, setpoint, self->F_d, self->M_d, tick, &self->desVirtInp, &self->desVirtInp_tick);
    }
    else {
      // DEBUG_PRINT("we are here\n");
      self->desVirtInp.x = self->F_d.x;
      self->desVirtInp.y = self->F_d.y;
      self->desVirtInp.z = self->F_d.z;
    }
    static int counter = 0;
    ++counter;
    #ifdef CRAZYFLIE_FW
      if (counter % 1000 == 0) {
        DEBUG_PRINT("db %f %f %f\n", (double)self->desVirtInp.x, (double)self->desVirtInp.y, (double)self->desVirtInp.z);
      }
    #endif
    
    // if we don't have a desVirtInp (yet), skip this 
    if (vmag2(self->desVirtInp) == 0) {
      // float force = vmag(vscl(1/state->num_uavs, self->F_d));
      // struct vec hover_force = mkvec(0,0,self->mp*GRAVITY_MAGNITUDE);
      // struct vec qi_tmp = vneg(vnormalize(vsub(plStPos, statePos)));
      // float force_i = vdot(hover_force, qi_tmp);
      // self->desVirtInp = vscl(force_i, qi_tmp);
      // self->desVirtInp.z = force;
      // DEBUG_PRINT("before QP db: %f %f %f\n", (double)self->desVirtInp.x, (double)self->desVirtInp.y, (double)self->desVirtInp.z);

      float force = 9.81f * (self->mp / (state->num_uavs));
      struct vec qi_tmp = vnormalize(vsub(plStPos, statePos));
      self->desVirtInp = vsclnorm(vneg(qi_tmp), force);
      // self->desVirtInp.z = force;
      DEBUG_PRINT("before QP db: %f %f %f\n", (double)self->desVirtInp.x, (double)self->desVirtInp.y, (double)self->desVirtInp.z);
      
      // return; 
    } else {
      // uint64_t t = usecTimestamp();
      // uint64_t delta = t - time_start;
      // DEBUG_PRINT("dvit %llu us\n", delta);
      // DEBUG_PRINT("db %f %f %f\n", (double)self->desVirtInp.x, (double)self->desVirtInp.y, (double)self->desVirtInp.z);
      // time_start = t;
    }

    // // if we don't have a desVirtInp (yet), estimate it based on the current cable state
    // if (vmag2(self->desVirtInp) == 0) {
    //   float force = 9.81f * (self->mass + self->mp / (1.0f + state->num_neighbors));
    //   self->desVirtInp = vsclnorm(vsub(plStPos, statePos), force); 
    // }
    // computed desired generalized forces in rigid payload case for equation 23 is Pmu_des = [Rp.T@F_d, M_d]
    // if a point mass for the payload is considered then: Pmu_des = F_d

    //directional unit vector qi and angular velocity wi pointing from UAV to payload
    self->qi = vnormalize(vsub(plStPos, statePos));

    // from the text between (2) and (3) in Lee's paper
    // qi_dot = (x0_dot + R0_dot rho_i - xi_dot)/li
    if (!isnanf(plquat.w)) {
      struct mat33 R0_dot = mmul(Rp, mcrossmat(plomega));
      self->qidot = vdiv(vsub(vadd(plStVel, mvmul(R0_dot, attPoint)), stateVel), l);
    } else {
      self->qidot = vdiv(vsub(plStVel, stateVel), l);
    }

    struct vec wi = vcross(self->qi, self->qidot);
    struct mat33 qiqiT = vecmult(self->qi);
    struct vec virtualInp = mvmul(qiqiT,self->desVirtInp);

    
    // Compute parallel component
    struct vec acc_ = plAcc_d;
    self->plAcc_des = plAcc_d;
    self->tension = 0;
    if (self->est_acc == 1) {
      struct vec plAcc_unfiltered = vzero();

      uint64_t timestamp_payload = usecTimestamp();
      float dt = (timestamp_payload - self->timestamp_payload_prev) / 1e6;

      plAcc_unfiltered = vdiv(vsub(plStVel, self->payload_vel_prev), dt);
      update_butterworth_2_low_pass_vec(filter_payload_acc, plAcc_unfiltered);
      self->plAcc_filtered = get_butterworth_2_low_pass_vec(filter_payload_acc);
      self->plAcc_filtered.z += GRAVITY_MAGNITUDE;
      // acc_ = self->plAcc_filtered; 

      self->payload_vel_prev = plStVel;
      self->timestamp_payload_prev = timestamp_payload;
    } else if (self->est_acc == 2) {
      // we want to compute Tq from  desired payload accelerations: self->plAcc_des, where self->plAcc_des has the gravity term
      self->tension = vdot(vscl(-self->mp, self->plAcc_des), self->qi);
      // self->Tq = vscl(tension, self->qi); // THIS IS THE ACCELERATION
      // acc_ = self->Tq;
    } 


    // struct vec acc_ = vscl(1/self->mp, self->F_d);
    if (!isnanf(plquat.w)) {
      if (self->en_accrb) {
        acc_ = vadd(plAcc_d, qvrot(plquat, mvmul(mmul(mcrossmat(plomega), mcrossmat(plomega)), attPoint)));
      }
    } 
    struct vec u_parallel = vadd3(virtualInp, vscl(self->mass*l*vmag2(wi), self->qi), vscl(self->mass, mvmul(qiqiT, acc_)));
    
    // Compute Perpindicular Component
    self->qdi = vneg(vnormalize(self->desVirtInp));
    struct vec eq  = vcross(self->qdi, self->qi);
    self->i_error_q = vadd(self->i_error_q, vscl(dt, eq));
    struct mat33 skewqi = mcrossmat(self->qi);
    struct mat33 skewqi2 = mmul(skewqi,skewqi);

    if (self->desVirtInp_tick != self->qdi_prev_tick) {
      if (self->en_qdidot) {
        self->qdidot = self->qid_ref;
      } else {
        self->qdidot = vzero();
      }
      self->qdi_prev = self->qdi;
      self->qdi_prev_tick = self->desVirtInp_tick;
    }
    struct vec wdi = vcross(self->qdi, self->qdidot);
    struct vec ew = vadd(wi, mvmul(skewqi2, wdi));

    // Lee's integral error (32)
    {
      struct vec part1 = vscl(1.0f/self->mp, vadd(plvel_e, vscl(self->c_x, plpos_e)));
      struct vec part2 = vzero();
      if (!isnanf(plquat.w)) {
        struct mat33 part2a = mmul(Rp, mcrossmat(attPoint));
        struct vec part2b = vadd(omega_perror, vscl(self->c_R, eRp));
        part2 = mvmul(part2a, part2b);
      }

      struct vec term1 = vscl(self->h_xi, mvmul(qiqiT, vsub(part1, part2)));

      struct vec term2 = vscl(self->h_xi / self->mass * l, mvmul(skewqi, vadd(ew, vscl(self->c_q, eq))));

      self->delta_bar_xi = vadd(self->delta_bar_xi, vscl(dt, vadd(term1, term2)));
    }

    // Lee (27)
    // this term ((vscl(vdot(self->qi, wdi), self->qidot)) is not negative in: 
    // https://ieeexplore.ieee.org/document/7040351 (not tested)
    struct vec u_perpind = vsub2(
      vscl(self->mass*l, mvmul(skewqi, vadd4(
        vneg(veltmul(self->K_q, vclampnorm(eq, self->K_q_limit))),
        vneg(veltmul(self->K_w, vclampnorm(ew, self->K_w_limit))),
        vneg(veltmul(self->K_q_I, self->i_error_q)), // main difference to Lee: Lee multiplies again by skewqui and normalizes by cable length
        vneg(vscl(vdot(self->qi, wdi), self->qidot))))),
      vscl(self->mass, mvmul(skewqi2, acc_)),
      vneg(mvmul(skewqi2, self->delta_bar_xi))
    );

    // current rotation [R]
    self->q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
    self->rpy = quat2rpy(self->q);
    self->R = quat2rotmat(self->q);

    if (self->use_nn) { 
      float start_time = usecTimestamp();
      // Acceleration ang gyroscope sensor readings
      self->input_vec[0] = sensors->acc.x;
      self->input_vec[1] = sensors->acc.y;
      self->input_vec[2] = sensors->acc.z;
      self->input_vec[3] = sensors->gyro.x;
      self->input_vec[4] = sensors->gyro.y;
      self->input_vec[5] = sensors->gyro.z;
      // First two columns of the rotation matrix
      self->input_vec[6] = self->R.m[0][0];
      self->input_vec[7] = self->R.m[0][1];
      self->input_vec[8] = self->R.m[1][0];
      self->input_vec[9] = self->R.m[1][1];
      self->input_vec[10] = self->R.m[2][0];
      self->input_vec[11] = self->R.m[2][1];
      const float *model_output = nn_forward(self->input_vec);
      self->nn_output[0] = model_output[0];
      self->nn_output[1] = model_output[1];
    
      float end_time = usecTimestamp();
      float elapsed_time = end_time - start_time;
      nn_inference_time_payload = elapsed_time; // Microseconds
    }

    struct vec a_nn = vzero();
    if (self->use_nn) {
      a_nn.x = self->nn_output[0] / self->mass;
      a_nn.y = self->nn_output[1] / self->mass;
    }

    // INDI
    struct vec a_indi = vzero();
    struct vec e3 = mkvec(0,0,1);
    if ((self->indi & 1) && rpm_deck_available) {

      float f_rpm = t1 + t2 + t3 + t4;
      // xdd_uav  = (f/m)Re3 - ge3 - (mp/m)*(xdd_payload + ge3)
      // note that acc_ = xdd_payload + ge3
      // add neural network:
      // self->a_rpm = vadd(vsub(vsub(vscl(f_rpm / self->mass, mvmul(self->R, e3)), mkvec(0.0, 0.0, 9.81f)), vscl(self->mp/self->mass, acc_)), a_nn);
      if (self->est_acc == 2) { 
      //   // (self->plAcc_des = xl_des + ge3) -->  Tq = -mp*self->plAcc_des --> tension = vdot(Tq, q) --> uav acc: xddot = (f/mass)Re3 - ge3 - Tq/mass, where Tq/mass = (tension/mass)q 
        self->a_rpm = vadd(vadd(vsub(vscl(f_rpm / self->mass, mvmul(self->R, e3)), mkvec(0.0, 0.0, 9.81f)), vscl(self->tension/ self->mass, self->qi)), a_nn);
      }
      // self->a_rpm = vsub(vscl(f_rpm / self->mass, mvmul(self->R, e3)), mkvec(0.0, 0.0, 9.81f));

      update_butterworth_2_low_pass_vec(filter_acc_rpm, self->a_rpm);

      // compute acceleration based on IMU (world frame, SI unit, no gravity)
      self->a_imu = vscl(9.81, mkvec(state->acc.x, state->acc.y, state->acc.z));
      update_butterworth_2_low_pass_vec(filter_acc_imu, self->a_imu);

      self->a_rpm_filtered = get_butterworth_2_low_pass_vec(filter_acc_rpm);
      self->a_imu_filtered = get_butterworth_2_low_pass_vec(filter_acc_imu);

      a_indi = vsub(self->a_rpm_filtered, self->a_imu_filtered);

      // DEBUG
      if (tick % 500 == 0) {
        DEBUG_PRINT("INDI p %f %f %f, %f %f %f\n", (double)self->a_rpm_filtered.x, (double)self->a_rpm_filtered.y, (double)self->a_rpm_filtered.z, (double)self->a_imu_filtered.x, (double)self->a_imu_filtered.y, (double)self->a_imu_filtered.z);
      }
    }


    self->u_i = vadd(u_parallel, u_perpind);
    // self->u_i = vsub(vsub(self->u_i, vscl(self->mass, a_indi)), a_nn);
    // u_i = u_parallel + u_perpind - (+?) mass*a_indi + mass*a_nn
    self->u_i = vadd(vsub(self->u_i, vscl(self->mass, a_indi)), vscl(self->mass, a_nn));

    // self->q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
    // self->rpy = quat2rpy(self->q);
    // self->R = quat2rotmat(self->q);
    // add neural network:
    control->thrustSi = vdot(self->u_i, mvmul(self->R, e3));

    // control->thrustSi = vmag(self->u_i);
    
    // control->u_all[0] = self->u_i.x;
    // control->u_all[1] = self->u_i.y;
    // control->u_all[2] = self->u_i.z;

    self->thrustSI = control->thrustSi;
    //  Reset the accumulated error while on the ground
    if (control->thrustSi < 0.01f) {
      controllerLeePayloadReset(self);
    }
  
    // Compute Desired Rotation matrix
    struct vec Fd_ = self->u_i;
    struct vec xdes = vbasis(0);
    struct vec ydes = vbasis(1);
    struct vec zdes = vbasis(2);
  
    if (self->thrustSI > 0) {
      zdes = vnormalize(Fd_);
    } 
    // struct vec xcdes = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0); 
    struct vec xcdes = mkvec(1, 0, 0); 
    struct vec zcrossx = vcross(zdes, xcdes);
    float normZX = vmag(zcrossx);

    if (normZX > 0) {
      ydes = vnormalize(zcrossx);
    } 
    xdes = vcross(ydes, zdes);
    
    self->R_des = mcolumns(xdes, ydes, zdes);
  } else {
    // DEBUG_PRINT("R2\n");
    // we only support position control
    control->controlMode = controlModeForceTorque;
    control->thrustSi  = 0;
    control->torque[0] = 0;
    control->torque[1] = 0;
    control->torque[2] = 0;
    controllerLeePayloadReset(self);
    return;
  }

  // Attitude controller

  // current rotation [R]
  self->q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  self->rpy = quat2rpy(self->q);
  self->R = quat2rotmat(self->q);

  // desired rotation [Rdes]
  struct quat q_des = mat2quat(self->R_des);
  self->rpy_des = quat2rpy(q_des);

  // rotation error
  struct mat33 eRM = msub(mmul(mtranspose(self->R_des), self->R), mmul(mtranspose(self->R), self->R_des));

  struct vec eR = vscl(0.5f, mkvec(eRM.m[2][1], eRM.m[0][2], eRM.m[1][0]));

  // angular velocity
  self->omega = mkvec(
    radians(sensors->gyro.x),
    radians(sensors->gyro.y),
    radians(sensors->gyro.z));

  // Compute desired omega
  if (self->en_num_omega) {
    if (self->desVirtInp_tick != self->prev_q_tick) {
      float q_dt = (self->desVirtInp_tick - self->prev_q_tick) / 1000.0f;
      self->omega_r = quat2omega(self->prev_q_des, q_des, q_dt);

      // static int counter = 0;
      // if (counter % 100 == 0) {
      // DEBUG_PRINT("or %f %f %f %f\n", (double)q_dt, (double)self->omega_r.x, (double)self->omega_r.y, (double)self->omega_r.z);
      // } 
      // ++ counter;
      self->prev_q_des = q_des;
      self->prev_q_tick = self->desVirtInp_tick;
    }
  } else {
    struct vec xdes = mcolumn(self->R_des, 0);
    struct vec ydes = mcolumn(self->R_des, 1);
    struct vec zdes = mcolumn(self->R_des, 2);
    struct vec hw = vzero();
    // Desired Jerk and snap for now are zeros vector
    struct vec desJerk = mkvec(setpoint->jerk.x, setpoint->jerk.y, setpoint->jerk.z);

    if (control->thrustSi != 0) {
      struct vec tmp = vsub(desJerk, vscl(vdot(zdes, desJerk), zdes));
      hw = vscl(self->mass/control->thrustSi, tmp);
    }

    struct vec z_w = mkvec(0, 0, 1);
    float desiredYawRate = radians(setpoint->attitudeRate.yaw) * vdot(zdes, z_w);
    struct vec omega_des = mkvec(-vdot(hw,ydes), vdot(hw,xdes), desiredYawRate);
    self->omega_r = mvmul(mmul(mtranspose(self->R), self->R_des), omega_des);
  }


  struct vec omega_error = vsub(self->omega, self->omega_r);
  
  // Integral part on angle
  self->i_error_att = vadd(self->i_error_att, vscl(dt, eR));

  // compute moments
  // M = -kR eR - kw ew + w x Jw - J(w x wr)
  self->u = vadd4(
    vneg(veltmul(self->KR, vclampnorm(eR, self->KR_limit))),
    vneg(veltmul(self->Komega, vclampnorm(omega_error, self->Komega_limit))),
    vneg(veltmul(self->KI, self->i_error_att)),
    vcross(self->omega, veltmul(self->J, self->omega)));

  // if (enableNN > 1) {
  //   u = vsub(u, tau_a);
  // }



  struct vec indi_moments;
  if ((self->indi & 2) && rpm_deck_available) {
    const float t2t = 0.006f;
    const float arm = 0.707106781f * 0.046f;
    self->tau_rpm = mkvec(
      -arm * t1 - arm * t2 + arm * t3 + arm * t4,
      -arm * t1 + arm * t2 + arm * t3 - arm * t4,
      -t2t * t1 + t2t * t2 - t2t * t3 + t2t * t4
    );
    update_butterworth_2_low_pass_vec(filter_tau_rpm, self->tau_rpm);

    self->tau_rpm_filtered = get_butterworth_2_low_pass_vec(filter_tau_rpm);

    // angular accelleration
    uint64_t timestamp = usecTimestamp();
    float dt = (timestamp - self->timestamp_prev) / 1e6;
    struct vec angular_acc = vdiv(vsub(self->omega, self->omega_prev), dt);
    self->tau_gyro = veltmul(self->J, angular_acc);

    update_butterworth_2_low_pass_vec(filter_angular_acc, angular_acc);

    struct vec angular_acc_filtered = get_butterworth_2_low_pass_vec(filter_angular_acc);
    self->tau_gyro_filtered = veltmul(self->J, angular_acc_filtered);

    self->omega_prev = self->omega;
    self->timestamp_prev = timestamp;

    indi_moments = vsub(self->tau_rpm_filtered, self->tau_gyro_filtered);
    indi_moments.z = 0.0f; // TODO: DEBUGGING ONLY DELETE ME
    self->u = vsub(self->u, indi_moments);

    // DEBUG
    if (tick % 1000 == 0) {
      DEBUG_PRINT("INDI a %f %f %f, %f %f %f\n", (double)self->tau_rpm_filtered.x, (double)self->tau_rpm_filtered.y, (double)self->tau_rpm_filtered.z, (double)self->tau_gyro_filtered.x, (double)self->tau_gyro_filtered.y, (double)self->tau_gyro_filtered.z);
    }
  }

  control->controlMode = controlModeForceTorque;
  control->torque[0] = self->u.x;
  control->torque[1] = self->u.y;
  control->torque[2] = self->u.z;

  // ticks = usecTimestamp() - startTime;
}

#ifdef CRAZYFLIE_FW

#include "param.h"
#include "log.h"

void controllerLeePayloadFirmwareTaskInit(void)
{
  if (!taskInitialized) {
    STATIC_MEM_TASK_CREATE(controllerLeePayloadQPTask, controllerLeePayloadQPTask, CONTROLLER_LEE_PAYLOAD_QP_TASK_NAME, NULL, CONTROLLER_LEE_PAYLOAD_QP_TASK_PRI);

    queueQPInput = STATIC_MEM_QUEUE_CREATE(queueQPInput);
    queueQPOutput = STATIC_MEM_QUEUE_CREATE(queueQPOutput);

    // struct QPOutput qpoutput;
    // memset(&qpoutput, 0, sizeof(qpoutput));
    // qpoutput.timestamp = 0;
    // qpoutput.desVirtInput = vzero();
    // xQueueOverwrite(queueQPOutput, &qpoutput);

    taskInitialized = true;
  }
}

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

// J
PARAM_ADD(PARAM_FLOAT, J_x, &g_self.J.x)
PARAM_ADD(PARAM_FLOAT, J_y, &g_self.J.y)
PARAM_ADD(PARAM_FLOAT, J_z, &g_self.J.z)

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

// Attitude Payload P
PARAM_ADD(PARAM_FLOAT, Kprot_Px, &g_self.Kprot_P.x)
PARAM_ADD(PARAM_FLOAT, Kprot_Py, &g_self.Kprot_P.y)
PARAM_ADD(PARAM_FLOAT, Kprot_Pz, &g_self.Kprot_P.z)
PARAM_ADD(PARAM_FLOAT, Kprot_P_limit, &g_self.Kprot_P_limit)
// Attitude Payload D
PARAM_ADD(PARAM_FLOAT, Kprot_Dx, &g_self.Kprot_D.x)
PARAM_ADD(PARAM_FLOAT, Kprot_Dy, &g_self.Kprot_D.y)
PARAM_ADD(PARAM_FLOAT, Kprot_Dz, &g_self.Kprot_D.z)
PARAM_ADD(PARAM_FLOAT, Kprot_D_limit, &g_self.Kprot_D_limit)
// Attitude Payload I
PARAM_ADD(PARAM_FLOAT, Kprot_Ix, &g_self.Kprot_I.x)
PARAM_ADD(PARAM_FLOAT, Kprot_Iy, &g_self.Kprot_I.y)
PARAM_ADD(PARAM_FLOAT, Kprot_Iz, &g_self.Kprot_I.z)
PARAM_ADD(PARAM_FLOAT, Kprot_I_limit, &g_self.Kprot_I_limit)


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

// UAV Attitude P
PARAM_ADD(PARAM_FLOAT, KRx, &g_self.KR.x)
PARAM_ADD(PARAM_FLOAT, KRy, &g_self.KR.y)
PARAM_ADD(PARAM_FLOAT, KRz, &g_self.KR.z)
PARAM_ADD(PARAM_FLOAT, KR_limit, &g_self.KR_limit)

// UAV Attitude D
PARAM_ADD(PARAM_UINT8, en_num_w, &g_self.en_num_omega)
PARAM_ADD(PARAM_FLOAT, Komx, &g_self.Komega.x)
PARAM_ADD(PARAM_FLOAT, Komy, &g_self.Komega.y)
PARAM_ADD(PARAM_FLOAT, Komz, &g_self.Komega.z)
PARAM_ADD(PARAM_FLOAT, Kom_limit, &g_self.Komega_limit)

// UAV Attitude I
PARAM_ADD(PARAM_FLOAT, KI_x, &g_self.KI.x)
PARAM_ADD(PARAM_FLOAT, KI_y, &g_self.KI.y)
PARAM_ADD(PARAM_FLOAT, KI_z, &g_self.KI.z)

PARAM_ADD(PARAM_UINT8, en_qdidot, &g_self.en_qdidot)
PARAM_ADD(PARAM_UINT8, en_accrb, &g_self.en_accrb)

PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)
PARAM_ADD(PARAM_FLOAT, massP, &g_self.mp)

PARAM_ADD(PARAM_FLOAT, radius, &g_self.radius)

// QP tuning

PARAM_ADD(PARAM_FLOAT, lambda, &g_self.lambdaa)
PARAM_ADD(PARAM_FLOAT, lambda_svm, &g_self.lambda_svm)

PARAM_ADD(PARAM_UINT8, gen_hp, &g_self.gen_hp)

PARAM_ADD(PARAM_UINT8, form_ctrl, &g_self.formation_control)

// Attachement points rigid body payload
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

// Pinv
PARAM_ADD(PARAM_UINT8, Pinv0id1,&g_self.Pinvs[0].id1)
PARAM_ADD(PARAM_UINT8, Pinv0id2,&g_self.Pinvs[0].id2)
PARAM_ADD(PARAM_FLOAT, Pinv000, &g_self.Pinvs[0].Pinv.m[0][0])
PARAM_ADD(PARAM_FLOAT, Pinv010, &g_self.Pinvs[0].Pinv.m[1][0])
PARAM_ADD(PARAM_FLOAT, Pinv020, &g_self.Pinvs[0].Pinv.m[2][0])
PARAM_ADD(PARAM_FLOAT, Pinv030, &g_self.Pinvs[0].Pinv.m[3][0])
PARAM_ADD(PARAM_FLOAT, Pinv040, &g_self.Pinvs[0].Pinv.m[4][0])
PARAM_ADD(PARAM_FLOAT, Pinv050, &g_self.Pinvs[0].Pinv.m[5][0])
PARAM_ADD(PARAM_FLOAT, Pinv001, &g_self.Pinvs[0].Pinv.m[0][1])
PARAM_ADD(PARAM_FLOAT, Pinv011, &g_self.Pinvs[0].Pinv.m[1][1])
PARAM_ADD(PARAM_FLOAT, Pinv021, &g_self.Pinvs[0].Pinv.m[2][1])
PARAM_ADD(PARAM_FLOAT, Pinv031, &g_self.Pinvs[0].Pinv.m[3][1])
PARAM_ADD(PARAM_FLOAT, Pinv041, &g_self.Pinvs[0].Pinv.m[4][1])
PARAM_ADD(PARAM_FLOAT, Pinv051, &g_self.Pinvs[0].Pinv.m[5][1])
PARAM_ADD(PARAM_FLOAT, Pinv002, &g_self.Pinvs[0].Pinv.m[0][2])
PARAM_ADD(PARAM_FLOAT, Pinv012, &g_self.Pinvs[0].Pinv.m[1][2])
PARAM_ADD(PARAM_FLOAT, Pinv022, &g_self.Pinvs[0].Pinv.m[2][2])
PARAM_ADD(PARAM_FLOAT, Pinv032, &g_self.Pinvs[0].Pinv.m[3][2])
PARAM_ADD(PARAM_FLOAT, Pinv042, &g_self.Pinvs[0].Pinv.m[4][2])
PARAM_ADD(PARAM_FLOAT, Pinv052, &g_self.Pinvs[0].Pinv.m[5][2])
PARAM_ADD(PARAM_FLOAT, Pinv003, &g_self.Pinvs[0].Pinv.m[0][3])
PARAM_ADD(PARAM_FLOAT, Pinv013, &g_self.Pinvs[0].Pinv.m[1][3])
PARAM_ADD(PARAM_FLOAT, Pinv023, &g_self.Pinvs[0].Pinv.m[2][3])
PARAM_ADD(PARAM_FLOAT, Pinv033, &g_self.Pinvs[0].Pinv.m[3][3])
PARAM_ADD(PARAM_FLOAT, Pinv043, &g_self.Pinvs[0].Pinv.m[4][3])
PARAM_ADD(PARAM_FLOAT, Pinv053, &g_self.Pinvs[0].Pinv.m[5][3])
PARAM_ADD(PARAM_FLOAT, Pinv004, &g_self.Pinvs[0].Pinv.m[0][4])
PARAM_ADD(PARAM_FLOAT, Pinv014, &g_self.Pinvs[0].Pinv.m[1][4])
PARAM_ADD(PARAM_FLOAT, Pinv024, &g_self.Pinvs[0].Pinv.m[2][4])
PARAM_ADD(PARAM_FLOAT, Pinv034, &g_self.Pinvs[0].Pinv.m[3][4])
PARAM_ADD(PARAM_FLOAT, Pinv044, &g_self.Pinvs[0].Pinv.m[4][4])
PARAM_ADD(PARAM_FLOAT, Pinv054, &g_self.Pinvs[0].Pinv.m[5][4])
PARAM_ADD(PARAM_FLOAT, Pinv005, &g_self.Pinvs[0].Pinv.m[0][5])
PARAM_ADD(PARAM_FLOAT, Pinv015, &g_self.Pinvs[0].Pinv.m[1][5])
PARAM_ADD(PARAM_FLOAT, Pinv025, &g_self.Pinvs[0].Pinv.m[2][5])
PARAM_ADD(PARAM_FLOAT, Pinv035, &g_self.Pinvs[0].Pinv.m[3][5])
PARAM_ADD(PARAM_FLOAT, Pinv045, &g_self.Pinvs[0].Pinv.m[4][5])
PARAM_ADD(PARAM_FLOAT, Pinv055, &g_self.Pinvs[0].Pinv.m[5][5])

PARAM_ADD(PARAM_UINT8, Pinv1id1,&g_self.Pinvs[1].id1)
PARAM_ADD(PARAM_UINT8, Pinv1id2,&g_self.Pinvs[1].id2)
PARAM_ADD(PARAM_FLOAT, Pinv100, &g_self.Pinvs[1].Pinv.m[0][0])
PARAM_ADD(PARAM_FLOAT, Pinv110, &g_self.Pinvs[1].Pinv.m[1][0])
PARAM_ADD(PARAM_FLOAT, Pinv120, &g_self.Pinvs[1].Pinv.m[2][0])
PARAM_ADD(PARAM_FLOAT, Pinv130, &g_self.Pinvs[1].Pinv.m[3][0])
PARAM_ADD(PARAM_FLOAT, Pinv140, &g_self.Pinvs[1].Pinv.m[4][0])
PARAM_ADD(PARAM_FLOAT, Pinv150, &g_self.Pinvs[1].Pinv.m[5][0])
PARAM_ADD(PARAM_FLOAT, Pinv101, &g_self.Pinvs[1].Pinv.m[0][1])
PARAM_ADD(PARAM_FLOAT, Pinv111, &g_self.Pinvs[1].Pinv.m[1][1])
PARAM_ADD(PARAM_FLOAT, Pinv121, &g_self.Pinvs[1].Pinv.m[2][1])
PARAM_ADD(PARAM_FLOAT, Pinv131, &g_self.Pinvs[1].Pinv.m[3][1])
PARAM_ADD(PARAM_FLOAT, Pinv141, &g_self.Pinvs[1].Pinv.m[4][1])
PARAM_ADD(PARAM_FLOAT, Pinv151, &g_self.Pinvs[1].Pinv.m[5][1])
PARAM_ADD(PARAM_FLOAT, Pinv102, &g_self.Pinvs[1].Pinv.m[0][2])
PARAM_ADD(PARAM_FLOAT, Pinv112, &g_self.Pinvs[1].Pinv.m[1][2])
PARAM_ADD(PARAM_FLOAT, Pinv122, &g_self.Pinvs[1].Pinv.m[2][2])
PARAM_ADD(PARAM_FLOAT, Pinv132, &g_self.Pinvs[1].Pinv.m[3][2])
PARAM_ADD(PARAM_FLOAT, Pinv142, &g_self.Pinvs[1].Pinv.m[4][2])
PARAM_ADD(PARAM_FLOAT, Pinv152, &g_self.Pinvs[1].Pinv.m[5][2])
PARAM_ADD(PARAM_FLOAT, Pinv103, &g_self.Pinvs[1].Pinv.m[0][3])
PARAM_ADD(PARAM_FLOAT, Pinv113, &g_self.Pinvs[1].Pinv.m[1][3])
PARAM_ADD(PARAM_FLOAT, Pinv123, &g_self.Pinvs[1].Pinv.m[2][3])
PARAM_ADD(PARAM_FLOAT, Pinv133, &g_self.Pinvs[1].Pinv.m[3][3])
PARAM_ADD(PARAM_FLOAT, Pinv143, &g_self.Pinvs[1].Pinv.m[4][3])
PARAM_ADD(PARAM_FLOAT, Pinv153, &g_self.Pinvs[1].Pinv.m[5][3])
PARAM_ADD(PARAM_FLOAT, Pinv104, &g_self.Pinvs[1].Pinv.m[0][4])
PARAM_ADD(PARAM_FLOAT, Pinv114, &g_self.Pinvs[1].Pinv.m[1][4])
PARAM_ADD(PARAM_FLOAT, Pinv124, &g_self.Pinvs[1].Pinv.m[2][4])
PARAM_ADD(PARAM_FLOAT, Pinv134, &g_self.Pinvs[1].Pinv.m[3][4])
PARAM_ADD(PARAM_FLOAT, Pinv144, &g_self.Pinvs[1].Pinv.m[4][4])
PARAM_ADD(PARAM_FLOAT, Pinv154, &g_self.Pinvs[1].Pinv.m[5][4])
PARAM_ADD(PARAM_FLOAT, Pinv105, &g_self.Pinvs[1].Pinv.m[0][5])
PARAM_ADD(PARAM_FLOAT, Pinv115, &g_self.Pinvs[1].Pinv.m[1][5])
PARAM_ADD(PARAM_FLOAT, Pinv125, &g_self.Pinvs[1].Pinv.m[2][5])
PARAM_ADD(PARAM_FLOAT, Pinv135, &g_self.Pinvs[1].Pinv.m[3][5])
PARAM_ADD(PARAM_FLOAT, Pinv145, &g_self.Pinvs[1].Pinv.m[4][5])
PARAM_ADD(PARAM_FLOAT, Pinv155, &g_self.Pinvs[1].Pinv.m[5][5])

PARAM_ADD(PARAM_UINT8, Pinv2id1,&g_self.Pinvs[2].id1)
PARAM_ADD(PARAM_UINT8, Pinv2id2,&g_self.Pinvs[2].id2)
PARAM_ADD(PARAM_FLOAT, Pinv200, &g_self.Pinvs[2].Pinv.m[0][0])
PARAM_ADD(PARAM_FLOAT, Pinv210, &g_self.Pinvs[2].Pinv.m[1][0])
PARAM_ADD(PARAM_FLOAT, Pinv220, &g_self.Pinvs[2].Pinv.m[2][0])
PARAM_ADD(PARAM_FLOAT, Pinv230, &g_self.Pinvs[2].Pinv.m[3][0])
PARAM_ADD(PARAM_FLOAT, Pinv240, &g_self.Pinvs[2].Pinv.m[4][0])
PARAM_ADD(PARAM_FLOAT, Pinv250, &g_self.Pinvs[2].Pinv.m[5][0])
PARAM_ADD(PARAM_FLOAT, Pinv201, &g_self.Pinvs[2].Pinv.m[0][1])
PARAM_ADD(PARAM_FLOAT, Pinv211, &g_self.Pinvs[2].Pinv.m[1][1])
PARAM_ADD(PARAM_FLOAT, Pinv221, &g_self.Pinvs[2].Pinv.m[2][1])
PARAM_ADD(PARAM_FLOAT, Pinv231, &g_self.Pinvs[2].Pinv.m[3][1])
PARAM_ADD(PARAM_FLOAT, Pinv241, &g_self.Pinvs[2].Pinv.m[4][1])
PARAM_ADD(PARAM_FLOAT, Pinv251, &g_self.Pinvs[2].Pinv.m[5][1])
PARAM_ADD(PARAM_FLOAT, Pinv202, &g_self.Pinvs[2].Pinv.m[0][2])
PARAM_ADD(PARAM_FLOAT, Pinv212, &g_self.Pinvs[2].Pinv.m[1][2])
PARAM_ADD(PARAM_FLOAT, Pinv222, &g_self.Pinvs[2].Pinv.m[2][2])
PARAM_ADD(PARAM_FLOAT, Pinv232, &g_self.Pinvs[2].Pinv.m[3][2])
PARAM_ADD(PARAM_FLOAT, Pinv242, &g_self.Pinvs[2].Pinv.m[4][2])
PARAM_ADD(PARAM_FLOAT, Pinv252, &g_self.Pinvs[2].Pinv.m[5][2])
PARAM_ADD(PARAM_FLOAT, Pinv203, &g_self.Pinvs[2].Pinv.m[0][3])
PARAM_ADD(PARAM_FLOAT, Pinv213, &g_self.Pinvs[2].Pinv.m[1][3])
PARAM_ADD(PARAM_FLOAT, Pinv223, &g_self.Pinvs[2].Pinv.m[2][3])
PARAM_ADD(PARAM_FLOAT, Pinv233, &g_self.Pinvs[2].Pinv.m[3][3])
PARAM_ADD(PARAM_FLOAT, Pinv243, &g_self.Pinvs[2].Pinv.m[4][3])
PARAM_ADD(PARAM_FLOAT, Pinv253, &g_self.Pinvs[2].Pinv.m[5][3])
PARAM_ADD(PARAM_FLOAT, Pinv204, &g_self.Pinvs[2].Pinv.m[0][4])
PARAM_ADD(PARAM_FLOAT, Pinv214, &g_self.Pinvs[2].Pinv.m[1][4])
PARAM_ADD(PARAM_FLOAT, Pinv224, &g_self.Pinvs[2].Pinv.m[2][4])
PARAM_ADD(PARAM_FLOAT, Pinv234, &g_self.Pinvs[2].Pinv.m[3][4])
PARAM_ADD(PARAM_FLOAT, Pinv244, &g_self.Pinvs[2].Pinv.m[4][4])
PARAM_ADD(PARAM_FLOAT, Pinv254, &g_self.Pinvs[2].Pinv.m[5][4])
PARAM_ADD(PARAM_FLOAT, Pinv205, &g_self.Pinvs[2].Pinv.m[0][5])
PARAM_ADD(PARAM_FLOAT, Pinv215, &g_self.Pinvs[2].Pinv.m[1][5])
PARAM_ADD(PARAM_FLOAT, Pinv225, &g_self.Pinvs[2].Pinv.m[2][5])
PARAM_ADD(PARAM_FLOAT, Pinv235, &g_self.Pinvs[2].Pinv.m[3][5])
PARAM_ADD(PARAM_FLOAT, Pinv245, &g_self.Pinvs[2].Pinv.m[4][5])
PARAM_ADD(PARAM_FLOAT, Pinv255, &g_self.Pinvs[2].Pinv.m[5][5])


// Lee's integral gains
PARAM_ADD(PARAM_FLOAT, h_x0, &g_self.h_x0)
PARAM_ADD(PARAM_FLOAT, h_R0, &g_self.h_R0)
PARAM_ADD(PARAM_FLOAT, h_xi, &g_self.h_xi)
PARAM_ADD(PARAM_FLOAT, c_x, &g_self.c_x)
PARAM_ADD(PARAM_FLOAT, c_R, &g_self.c_R)
PARAM_ADD(PARAM_FLOAT, c_q, &g_self.c_q)

// Manual rotation (rad)
PARAM_ADD(PARAM_FLOAT, roll_des, &g_self.roll_des)
PARAM_ADD(PARAM_FLOAT, pitch_des, &g_self.pitch_des)


// INDI
PARAM_ADD(PARAM_UINT8, indi, &g_self.indi)
PARAM_ADD(PARAM_UINT8, est_acc, &g_self.est_acc)
PARAM_ADD(PARAM_UINT8, use_nn, &g_self.use_nn)


PARAM_GROUP_STOP(ctrlLeeP)


LOG_GROUP_START(ctrlLeeP)

LOG_ADD(LOG_FLOAT, thrustSI, &g_self.thrustSI)
LOG_ADD(LOG_FLOAT, torquex, &g_self.u.x)
LOG_ADD(LOG_FLOAT, torquey, &g_self.u.y)
LOG_ADD(LOG_FLOAT, torquez, &g_self.u.z)

// current angles
LOG_ADD(LOG_FLOAT, rpyx, &g_self.rpy.x)
LOG_ADD(LOG_FLOAT, rpyy, &g_self.rpy.y)
LOG_ADD(LOG_FLOAT, rpyz, &g_self.rpy.z)

// desired angles
LOG_ADD(LOG_FLOAT, rpydx, &g_self.rpy_des.x)
LOG_ADD(LOG_FLOAT, rpydy, &g_self.rpy_des.y)
LOG_ADD(LOG_FLOAT, rpydz, &g_self.rpy_des.z)

// desired quat for the payload
LOG_ADD(LOG_FLOAT, qp_desx, &g_self.qp_des.x)
LOG_ADD(LOG_FLOAT, qp_desy, &g_self.qp_des.y)
LOG_ADD(LOG_FLOAT, qp_desz, &g_self.qp_des.z)
LOG_ADD(LOG_FLOAT, qp_desw, &g_self.qp_des.w)

// desired omega for payload
LOG_ADD(LOG_FLOAT, omega_prx, &g_self.omega_pr.x)
LOG_ADD(LOG_FLOAT, omega_pry, &g_self.omega_pr.y)
LOG_ADD(LOG_FLOAT, omega_prz, &g_self.omega_pr.z)

// omega
LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

LOG_ADD(LOG_FLOAT, ux, &g_self.u_i.x)
LOG_ADD(LOG_FLOAT, uy, &g_self.u_i.y)
LOG_ADD(LOG_FLOAT, uz, &g_self.u_i.z)
// Cable States
LOG_ADD(LOG_FLOAT, qix, &g_self.qi.x)
LOG_ADD(LOG_FLOAT, qiy, &g_self.qi.y)
LOG_ADD(LOG_FLOAT, qiz, &g_self.qi.z)

LOG_ADD(LOG_FLOAT, qidotx, &g_self.qidot.x)
LOG_ADD(LOG_FLOAT, qidoty, &g_self.qidot.y)
LOG_ADD(LOG_FLOAT, qidotz, &g_self.qidot.z)

LOG_ADD(LOG_FLOAT, qdidotx, &g_self.qdidot.x)
LOG_ADD(LOG_FLOAT, qdidoty, &g_self.qdidot.y)
LOG_ADD(LOG_FLOAT, qdidotz, &g_self.qdidot.z)

// Cable States
LOG_ADD(LOG_FLOAT, qdix, &g_self.qdi.x)
LOG_ADD(LOG_FLOAT, qdiy, &g_self.qdi.y)
LOG_ADD(LOG_FLOAT, qdiz, &g_self.qdi.z)


LOG_ADD(LOG_FLOAT, qidrefx, &g_self.qid_ref.x)
LOG_ADD(LOG_FLOAT, qidrefy, &g_self.qid_ref.y)
LOG_ADD(LOG_FLOAT, qidrefz, &g_self.qid_ref.z)

// hyperplanes
LOG_ADD(LOG_FLOAT, n1x, &g_self.n[0].x)
LOG_ADD(LOG_FLOAT, n1y, &g_self.n[0].y)
LOG_ADD(LOG_FLOAT, n1z, &g_self.n[0].z)

LOG_ADD(LOG_FLOAT, n2x, &g_self.n[1].x)
LOG_ADD(LOG_FLOAT, n2y, &g_self.n[1].y)
LOG_ADD(LOG_FLOAT, n2z, &g_self.n[1].z)

// computed desired payload force
LOG_ADD(LOG_FLOAT, Fdx, &g_self.F_d.x)
LOG_ADD(LOG_FLOAT, Fdy, &g_self.F_d.y)
LOG_ADD(LOG_FLOAT, Fdz, &g_self.F_d.z)

LOG_ADD(LOG_FLOAT, Mdx, &g_self.M_d.x)
LOG_ADD(LOG_FLOAT, Mdy, &g_self.M_d.y)
LOG_ADD(LOG_FLOAT, Mdz, &g_self.M_d.z)

// computed virtual input
LOG_ADD(LOG_FLOAT, desVirtInpx, &g_self.desVirtInp.x)
LOG_ADD(LOG_FLOAT, desVirtInpy, &g_self.desVirtInp.y)
LOG_ADD(LOG_FLOAT, desVirtInpz, &g_self.desVirtInp.z)


// computed virtual input
LOG_ADD(LOG_FLOAT, mu_refx, &g_self.mu_ref.x)
LOG_ADD(LOG_FLOAT, mu_refy, &g_self.mu_ref.y)
LOG_ADD(LOG_FLOAT, mu_refz, &g_self.mu_ref.z)


// LOG_ADD(LOG_FLOAT, desCableVecx, &g_self.desiredCableUnitVec[0].x)
// LOG_ADD(LOG_FLOAT, desCableVecy, &g_self.desiredCableUnitVec[0].y)
// LOG_ADD(LOG_FLOAT, desCableVecz, &g_self.desiredCableUnitVec[0].z)


// INDI
LOG_ADD(LOG_FLOAT, f_rpm, &g_self.f_rpm)         // compare to thrustSi
LOG_ADD(LOG_FLOAT, tau_rpmx, &g_self.tau_rpm.x)  // compare to torquex
LOG_ADD(LOG_FLOAT, tau_rpmy, &g_self.tau_rpm.y)  // compare to torquey
LOG_ADD(LOG_FLOAT, tau_rpmz, &g_self.tau_rpm.z)  // compare to torquez

LOG_ADD(LOG_FLOAT, tau_rpm_fx, &g_self.tau_rpm_filtered.x)  // compare to torquex
LOG_ADD(LOG_FLOAT, tau_rpm_fy, &g_self.tau_rpm_filtered.y)  // compare to torquey
LOG_ADD(LOG_FLOAT, tau_rpm_fz, &g_self.tau_rpm_filtered.z)  // compare to torquez

LOG_ADD(LOG_FLOAT, tau_gyro_x, &g_self.tau_gyro.x)  // compare to torquex
LOG_ADD(LOG_FLOAT, tau_gyro_y, &g_self.tau_gyro.y)  // compare to torquey
LOG_ADD(LOG_FLOAT, tau_gyro_z, &g_self.tau_gyro.z)  // compare to torquez

LOG_ADD(LOG_FLOAT, tau_gyro_fx, &g_self.tau_gyro_filtered.x)  // compare to torquex
LOG_ADD(LOG_FLOAT, tau_gyro_fy, &g_self.tau_gyro_filtered.y)  // compare to torquey
LOG_ADD(LOG_FLOAT, tau_gyro_fz, &g_self.tau_gyro_filtered.z)  // compare to torquez

LOG_ADD(LOG_FLOAT, a_rpmx, &g_self.a_rpm.x)
LOG_ADD(LOG_FLOAT, a_rpmy, &g_self.a_rpm.y)
LOG_ADD(LOG_FLOAT, a_rpmz, &g_self.a_rpm.z)

LOG_ADD(LOG_FLOAT, a_rpm_fx, &g_self.a_rpm_filtered.x)
LOG_ADD(LOG_FLOAT, a_rpm_fy, &g_self.a_rpm_filtered.y)
LOG_ADD(LOG_FLOAT, a_rpm_fz, &g_self.a_rpm_filtered.z)

LOG_ADD(LOG_FLOAT, a_imux, &g_self.a_imu.x)
LOG_ADD(LOG_FLOAT, a_imuy, &g_self.a_imu.y)
LOG_ADD(LOG_FLOAT, a_imuz, &g_self.a_imu.z)

LOG_ADD(LOG_FLOAT, a_imu_fx, &g_self.a_imu_filtered.x)
LOG_ADD(LOG_FLOAT, a_imu_fy, &g_self.a_imu_filtered.y)
LOG_ADD(LOG_FLOAT, a_imu_fz, &g_self.a_imu_filtered.z)

// Acceleration Estimation

LOG_ADD(LOG_FLOAT, plAccx, &g_self.plAcc_filtered.x)
LOG_ADD(LOG_FLOAT, plAccy, &g_self.plAcc_filtered.y)
LOG_ADD(LOG_FLOAT, plAccz, &g_self.plAcc_filtered.z)

LOG_ADD(LOG_FLOAT, plAccx_des, &g_self.plAcc_des.x)
LOG_ADD(LOG_FLOAT, plAccy_des, &g_self.plAcc_des.y)
LOG_ADD(LOG_FLOAT, plAccz_des, &g_self.plAcc_des.z)

LOG_ADD(LOG_FLOAT, plAccx_tq, &g_self.Tq.x)
LOG_ADD(LOG_FLOAT, plAccy_tq, &g_self.Tq.y)
LOG_ADD(LOG_FLOAT, plAccz_tq, &g_self.Tq.z)


LOG_ADD(LOG_FLOAT, plVelx, &g_self.plVel_filtered.x)
LOG_ADD(LOG_FLOAT, plVely, &g_self.plVel_filtered.y)
LOG_ADD(LOG_FLOAT, plVelz, &g_self.plVel_filtered.z)

LOG_ADD(LOG_UINT32, profQP, &qp_runtime_us)

// NN

LOG_ADD(LOG_FLOAT, nnfx, &g_self.nn_output[0])
LOG_ADD(LOG_FLOAT, nnfy, &g_self.nn_output[1])

LOG_GROUP_STOP(ctrlLeeP)

#endif // CRAZYFLIE_FW defined


               


          
