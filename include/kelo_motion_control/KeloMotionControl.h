#ifndef KELO_MOTION_CONTROL_H
#define KELO_MOTION_CONTROL_H

#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_sf_trig.h>
#include <stdbool.h>

#include "kelo_motion_control/PlatformToWheelSolver.h"

#define MOTOR_CONST 3.5714

typedef struct
{
  gsl_matrix *A;
  gsl_matrix *A_inv_T;
  gsl_matrix *A_tmp;
  gsl_matrix *A_inv_T_tmp;
  gsl_vector *work;
  gsl_matrix *W;
  gsl_matrix *K;
  gsl_vector *u;
  gsl_matrix *V;
  gsl_matrix *u_inv;
  gsl_matrix *b;
  gsl_matrix *b_verify;
} TorqueControlState;

void init_kelo_base_config(KeloBaseConfig *config, int nWheels, int *index_to_EtherCAT,
                           double radius, double castor_offset,
                           double half_wheel_distance, double *wheel_coordinates,
                           double *pivot_angles_deviation);

void init_torque_control_state(TorqueControlState *state, int N, int M);

void set_weight_matrix(TorqueControlState *state, int N, int M);

void set_platform_force(TorqueControlState *state, double *platform_force);

void compute_wheel_torques(KeloBaseConfig *config, TorqueControlState *state, double *pivot_angles, double *wheel_torques,
                           int N, int M);

void free_torque_control_state(TorqueControlState *state);

#endif  // KELO_MOTION_CONTROL_H