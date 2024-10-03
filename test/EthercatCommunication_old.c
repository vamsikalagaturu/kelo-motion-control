/**
 * @file EthercatCommunication.c
 * @author Kavya Shankar (kavya.shankar@smail.inf.h-brs.de)
 * @brief Establishing connection with ehtercat and performing data transfer with robot
 * @date 2022-03-12
 *
 */
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_sf_trig.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"
#include "soem/ethercat.h"
#include "soem/ethercatbase.h"
#include "soem/ethercatcoe.h"
#include "soem/ethercatconfig.h"
#include "soem/ethercatdc.h"
#include "soem/ethercatmain.h"
#include "soem/ethercatprint.h"
#include "soem/ethercattype.h"
#include "soem/nicdrv.h"

/**
 * @brief Establishing connection with ehtercat and performing data transfer with robot
 *
 * @param argc
 * @param argv
 * @return int to signify successful execution of the function
 */
int main(int argc, char *argv[])
{
  ec_slavet ecx_slave[EC_MAXSLAVE];
  int ecx_slavecount;
  ec_groupt ec_group[EC_MAXGROUP];
  uint8 esibuf[EC_MAXEEPBUF];
  uint32 esimap[EC_MAXEEPBITMAP];
  ec_eringt ec_elist;
  ec_idxstackT ec_idxstack;

  ec_SMcommtypet ec_SMcommtype;
  ec_PDOassignt ec_PDOassign;
  ec_PDOdesct ec_PDOdesc;
  ec_eepromSMt ec_SM;
  ec_eepromFMMUt ec_FMMU;
  boolean EcatError;
  int64 ec_DCtime;
  ecx_portt ecx_port;
  ecx_redportt ecx_redport;
  ecx_contextt ecx_context;
  char IOmap[4096];

  ecx_context.port = &ecx_port;
  ecx_context.slavelist = &ecx_slave[0];
  ecx_context.slavecount = &ecx_slavecount;
  ecx_context.maxslave = EC_MAXSLAVE;
  ecx_context.grouplist = &ec_group[0];
  ecx_context.maxgroup = EC_MAXGROUP;
  ecx_context.esibuf = &esibuf[0];
  ecx_context.esimap = &esimap[0];
  ecx_context.esislave = 0;
  ecx_context.elist = &ec_elist;
  ecx_context.idxstack = &ec_idxstack;

  ecx_context.ecaterror = &EcatError;
  ecx_context.DCtime = &ec_DCtime;
  ecx_context.SMcommtype = &ec_SMcommtype;
  ecx_context.PDOassign = &ec_PDOassign;
  ecx_context.PDOdesc = &ec_PDOdesc;
  ecx_context.eepSM = &ec_SM;
  ecx_context.eepFMMU = &ec_FMMU;
  ecx_context.manualstatechange = 0;  // should be 0

  int nWheels = 4;
  int index_to_EtherCAT[4] = {6, 7, 3, 4};
  bool debug = false;

  /**
   * @brief port name on our PC to initiate connection
   *
   */
  if (!ecx_init(&ecx_context, "eno1"))
  {
    printf("Failed to initialize EtherCAT\n");
    return 0;
  }

  printf("EtherCAT initialized\n");

  /**
   * @brief checking establishment of first connection with slave or autoconfig slaves
   *
   */
  if (!ecx_config_init(&ecx_context, TRUE))
  {
    printf("NO SLAVES!\n");
    return 0;
  }
  ecx_config_map_group(&ecx_context, IOmap, 0);  // PDO - process data object

  printf("%i\n", ecx_slavecount);
  printf("%s\n", ecx_slave[1].name);

  /**
   * @brief Reading all slave names w.r.t their no.
   *
   */
  for (int i = 1; i <= ecx_slavecount; i++)
  {
    printf("slave \t%i has name \t%s\n", i, ecx_slave[i].name);
  }

  /**
   * @brief waiting for all slaves to reach SAFE_OP state
   *
   */
  ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

  if (ecx_slave[0].state != EC_STATE_SAFE_OP)
  {
    printf("EtherCAT slaves have not reached safe operational state\n");
    ecx_readstate(&ecx_context);

    /**
     * @brief if not all slaves operational, find out which one
     *
     */
    for (int i = 1; i <= ecx_slavecount; i++)
    {
      if (ecx_slave[i].state != EC_STATE_SAFE_OP)
      {
        printf("Slave %i State= %i\n", i, ecx_slave[i].state);
      }
    }
    return 0;
  }

  rxpdo1_t msg;
  msg.timestamp = 1;
  msg.command1 = 0;
  msg.limit1_p = 0;
  msg.limit1_n = 0;
  msg.limit2_p = 0;
  msg.limit2_n = 0;
  msg.setpoint1 = 0;
  msg.setpoint2 = 0;
  
  for (unsigned int i = 0; i < nWheels; i++)
  {
    rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
    *ecData = msg;
  }

  /**
   * @brief sending process data
   *
   */
  ecx_send_processdata(&ecx_context);

  /**
   * @brief setting state to operational
   *
   */
  ecx_slave[0].state = EC_STATE_OPERATIONAL;

  /**
   * @brief receiving response from slaves
   *
   */
  ecx_send_processdata(&ecx_context);
  ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

  ecx_writestate(&ecx_context, 0);

  /**
   * @brief checking if the slaves have reached operational state
   *
   */
  ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

  if (ecx_slave[0].state != EC_STATE_OPERATIONAL)
  {
    printf("EtherCAT slaves have not reached operational state\n");
    return 0;
  }
  else
  {
    printf("Operational state reached for all EtherCAT slaves.\n");
  }

  /**
   * @brief initialising pointers to variables used for solving the problem of inverse
   * kinematics
   *
   */
  int cnt = 0;
  const unsigned int N = 3;
  const unsigned int M = 8;
  double motor_const = 3.5714;  // units: (Ampere/Newton-meter)
  gsl_matrix *A = gsl_matrix_alloc(N, M);
  gsl_matrix *A_inv_T = gsl_matrix_alloc(M, N);
  gsl_matrix *A_tmp = gsl_matrix_alloc(N, M);
  gsl_matrix *A_inv_T_tmp = gsl_matrix_alloc(M, N);
  gsl_vector *work = gsl_vector_alloc(N);
  gsl_matrix *W = gsl_matrix_alloc(N, N);
  gsl_matrix *K = gsl_matrix_alloc(M, M);
  gsl_vector *u = gsl_vector_alloc(N);
  gsl_matrix *V = gsl_matrix_alloc(N, N);
  gsl_matrix *u_inv = gsl_matrix_alloc(N, N);
  gsl_matrix *b = gsl_matrix_alloc(N, 1);
  gsl_matrix *b_verify = gsl_matrix_alloc(N, 1);

  /**
   * @brief initialising arrays to store pivot angles and wheel torques
   *
   */
  double pivot_angles[4];
  double wheel_torques[8];

  /**
   * @brief setting input platform force values
   *
   */
  gsl_matrix_set(b, 0, 0, 0.);  // force is set in X-direction
  gsl_matrix_set(b, 1, 0, 0.);   // force is set in Y-direction
  gsl_matrix_set(b, 2, 0, 3.);   // moment is set in anti-clockwise direction

  double pivot_angles_deviation[4] = {-2.5, -1.25, -2.14, 1.49};

  /**
   * @brief reading data from individual wheels
   *
   */
  for (unsigned int i = 0; i < nWheels; i++)
  {
    txpdo1_t *ecData = (txpdo1_t *)ecx_slave[index_to_EtherCAT[i]].inputs;
    pivot_angles[i] = ecData->encoder_pivot - pivot_angles_deviation[i];
    if (pivot_angles[i] > 2 * M_PI)
      pivot_angles[i] -= 2 * M_PI;
    else if (pivot_angles[i] < 0.0)
      pivot_angles[i] += 2 * M_PI;
  }

  /**
   * @brief setting the weght matrix
   *
   */
  size_t i;
  for (i = 0; i < M; i++)
  {
    gsl_matrix_set(K, i, i, 1.0);
    if (i < N)
    {
      gsl_matrix_set(W, i, i, 1.0);
    }
  }

  KeloBaseConfig kelo_base_config;
  double radius = 0.052;
  double castor_offset = 0.01;
  double half_wheel_distance = 0.0275;
  double wheel_coordinates[8] = {0.175,  0.1605,  -0.175, 0.1605,
                                 -0.175, -0.1605, 0.175,  -0.1605};  // x1,y1,x2,y2,..,y4
  
  init_kelo_base_config(&kelo_base_config, nWheels, index_to_EtherCAT, radius,
                        castor_offset, half_wheel_distance, wheel_coordinates,
                        pivot_angles_deviation);

  /**
   * @brief setting number of iterations until which the force has to be applied
   *
   */
  printf("Starting loop\n");
  while (true)
  {
    printf("Iteration %d\n", cnt);
    /**
     * @brief setting sleep time between iterations to achieve communication frequency of
     * 1000Hz
     *
     */
    // usleep(10000);

    /**
     * @brief finding wheel torques for each iteration parameterised by pivot angles
     *
     */
    platform_force_to_wheel_torques(&kelo_base_config, wheel_torques, pivot_angles, b,
                                    b_verify, A, A_inv_T, A_tmp, A_inv_T_tmp, work, W, K,
                                    u, V, u_inv, M, N, debug);
    cnt += 1;
    rxpdo1_t msg;
    msg.timestamp = time(NULL);
    msg.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_TORQUE;
    msg.limit1_p = 3;   // upper limit for first wheel
    msg.limit1_n = -3;  // lower limit for first wheel
    msg.limit2_p = 3;   // upper limit for second wheel
    msg.limit2_n = -3;  // lower limit for second wheel

    /**
     * @brief setting calculated torque values to individual wheels
     *
     */
    if (debug)
    {
      printf("\nsetpoint values:\n");
    }

    for (unsigned int i = 0; i < nWheels; i++)  // runs all wheels
    {
      msg.setpoint1 = -motor_const * wheel_torques[2 * i];  // units: (rad/sec)
      msg.setpoint2 = motor_const * wheel_torques[2 * i + 1];
      rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
      *ecData = msg;

      /**
       * @brief printing angles after offsetting the pivots
       *
       */
      if (debug)
      {
        printf("%f\t", -motor_const * wheel_torques[2 * i]);
        printf("%f\t", motor_const * wheel_torques[2 * i + 1]);
      }
    }

    /**
     * @brief Construct a new ecx send processdata object
     *
     */
    ecx_send_processdata(&ecx_context);

    /**
     * @brief Construct a new ecx receive processdata object
     *
     */
    ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

    /**
     * @brief receiving updated pivot angles
     *
     */
    for (unsigned int i = 0; i < nWheels; i++)
    {
      txpdo1_t *ecData = (txpdo1_t *)ecx_slave[index_to_EtherCAT[i]].inputs;
      pivot_angles[i] = ecData->encoder_pivot - pivot_angles_deviation[i];
      if (pivot_angles[i] > 2 * M_PI)
        pivot_angles[i] -= 2 * M_PI;
      else if (pivot_angles[i] < 0.0)
        pivot_angles[i] += 2 * M_PI;
    }
  }

  /**
   * @brief releasing memory from all initialised pointers
   *
   */
  gsl_matrix_free(b);
  gsl_matrix_free(b_verify);

  gsl_matrix_free(A);
  gsl_matrix_free(A_inv_T);
  gsl_matrix_free(A_tmp);
  gsl_matrix_free(A_inv_T_tmp);
  gsl_matrix_free(W);
  gsl_matrix_free(K);
  gsl_vector_free(u);
  gsl_matrix_free(u_inv);
  gsl_matrix_free(V);
  gsl_vector_free(work);

  return 0;
}
