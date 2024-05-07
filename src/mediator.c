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

void establish_kelo_base_connection(KeloBaseConfig* kelo_base_config, EthercatConfig* ethercat_config, char* ifname,
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

  rxpdo1_t msg;
  msg.timestamp = 1;
  msg.command1 = 0;
  msg.limit1_p = 0;
  msg.limit1_n = 0;
  msg.limit2_p = 0;
  msg.limit2_n = 0;
  msg.setpoint1 = 0;
  msg.setpoint2 = 0;
  
  for (unsigned int i = 0; i < kelo_base_config->nWheels; i++)
  {
    rxpdo1_t *ecData = (rxpdo1_t *)ethercat_config->ecx_slave[kelo_base_config->index_to_EtherCAT[i]].outputs;
    *ecData = msg;
  }

  process_data_exchange(ethercat_config);
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
                    kelo_base_config->nWheels, kelo_base_config->pivot_angles_deviation);
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