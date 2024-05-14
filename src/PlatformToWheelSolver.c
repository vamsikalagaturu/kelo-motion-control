/**
 * @file PlatformToWheelInverseKinematicsSolver.c
 * @author Sivva Rahul Sai (rahul.sivva@smail.inf.h-brs.de)
 * @brief this file consists of a functionality to print a matrix and to integrate all functions
 * related to calculating torques at individual wheel units
 * @date 2022-03-12
 *
 */

#include "kelo_motion_control/PlatformToWheelSolver.h"

#define _USE_MATH_DEFINES

void print_matrix(const gsl_matrix *m)
{
  size_t i, j;

  for (i = 0; i < m->size1; i++)
  {
    for (j = 0; j < m->size2; j++)
    {
      printf("%f\t", gsl_matrix_get(m, i, j));
    }
  }
  printf("\n");
}

void platform_force_to_wheel_torques(KeloBaseConfig *config, double *wheel_torques, double *pivot_angles,
                                     const gsl_matrix *b, gsl_matrix *b_verify, gsl_matrix *A,
                                     gsl_matrix *A_inv_T, gsl_matrix *A_tmp,
                                     gsl_matrix *A_inv_T_tmp, gsl_vector *work,
                                     const gsl_matrix *W, const gsl_matrix *K, gsl_vector *u,
                                     gsl_matrix *V, gsl_matrix *u_inv, const unsigned int M,
                                     const unsigned int N, const bool debug)
{
  /**
   * @brief 1. initialise robot geometrical parameters
   * https://github.com/kelo-robotics/kelo_tulip/blob/73e6d134bd31da6c580846dc907ff1ce2565b406/src/VelocityPlatformController.cpp
   * https://github.com/kelo-robotics/kelo_tulip/blob/master/src/PlatformDriver.cpp
   *
   */

  double radius = config->radius;
  double castor_offset = config->castor_offset;
  double half_wheel_distance = config->half_wheel_distance;
  double *wheel_coordinates = config->wheel_coordinates;

  double pivot_forces[config->nWheels * 2];
  for (int i = 0; i < config->nWheels * 2; i++)
  {
    pivot_forces[i] = 0.0;
  }

  /**
   * @brief 2. get jacobian (A) -> KELORobotKinematics.c
   *
   */
  jacobian_matrix_calculator(A, pivot_angles, wheel_coordinates);

  /**
   * @brief 3. find force array (pivot_forces) -> KELORobotKinematics.c
   *
   */
  force_vector_finder(pivot_forces, A, A_tmp, A_inv_T_tmp, A_inv_T, u, u_inv, V, W, K, work, b, M);

  /**
   * @brief 4. find torques at individual wheels (wheel_torques) -> SmartWheelKinematics.c
   *
   */
  map_pivot_forces_to_wheel_torques(pivot_forces, wheel_torques, radius, castor_offset,
                                    half_wheel_distance);

  /**
   * @brief 5. to print results for debugging
   *
   */
  if (debug)
  {
    /**
     * @brief printing angles after offsetting the pivots
     *
     */
    printf("\n\n\n\nPivot angles:\n");
    for (int i = 0; i < 4; i++)
    {
      printf("%f\t", pivot_angles[i]);
    }

    printf("\nPivot forces:\n");
    for (int i = 0; i < 8; i++)
    {
      printf("%f\t", pivot_forces[i]);
    }

    printf("\nWheel torques:\n");

    for (int i = 0; i < 8; i++)
    {
      printf("%f\t", wheel_torques[i]);
    }
    gsl_matrix_view _Y = gsl_matrix_view_array(pivot_forces, M, 1);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, &_Y.matrix, 0.0, b_verify);
    printf("\nReverse calculation: Platform force: \n");
    print_matrix(b_verify);
    print_matrix(b);
  }
}
