#include "kelo_motion_control/mediator.h"

void initialize_kelo_base(KeloBaseConfig* kelo_base_config,
                          EthercatConfig* ethercat_config)
{
  int nWheels = 4;
  int index_to_EtherCAT[4] = {2, 3, 5, 6};
  double radius = 0.052;
  double castor_offset = 0.01;
  double half_wheel_distance = 0.0275;
  double wheel_coordinates[8] = {0.175,  0.1605,  -0.175, 0.1605,
                                 -0.175, -0.1605, 0.175,  -0.1605};  // x1,y1,x2,y2,..,y4
  double pivot_angles_deviation[4] = {-2.5, -1.25, -2.14, 1.49};
  init_kelo_base_config(kelo_base_config, nWheels, index_to_EtherCAT, radius,
                        castor_offset, half_wheel_distance, wheel_coordinates,
                        pivot_angles_deviation);

  init_ecx_context(ethercat_config);
}

void establish_kelo_base_connection(EthercatConfig* ethercat_config, char* ifname,
                                    int* result)
{
  *result = 0;

  establish_connection(ethercat_config, ifname, result);
  if (*result == -1)
  {
    printf("Failed to establish connection\n");
    return;
  }

  check_slave_state(ethercat_config, EC_STATE_SAFE_OP, result);
  if (*result == -1)
  {
    printf("EtherCAT slaves have not reached safe operational state\n");
    return;
  }

  process_data_exchange(ethercat_config, false);
  check_slave_state(ethercat_config, EC_STATE_OPERATIONAL, result);
  if (*result == -1)
  {
    printf("EtherCAT slaves have not reached operational state\n");
    return;
  }
}

void get_kelo_base_state(KeloBaseConfig* kelo_base_config,
                         EthercatConfig* ethercat_config, double* pivot_angles)
{
  read_pivot_angles(ethercat_config, pivot_angles, kelo_base_config->index_to_EtherCAT,
                    kelo_base_config->nWheels);
}

void set_kelo_base_torques(KeloBaseConfig* kelo_base_config,
                           EthercatConfig* ethercat_config, double* wheel_torques)
{
  rxpdo1_t rx_msg;
  create_rx_msg(&rx_msg);
  set_wheel_torques(ethercat_config, &rx_msg, kelo_base_config->index_to_EtherCAT,
                    wheel_torques, kelo_base_config->nWheels, MOTOR_CONST);
  send_and_receive_data(ethercat_config);
}