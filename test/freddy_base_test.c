#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"

int main(int argc, char *argv[])
{
  int nWheels = 4;
  int index_to_EtherCAT[4] = {2, 3, 5, 6};
  KeloBaseConfig kelo_base_config;
  double radius = 0.052;
  double castor_offset = 0.01;
  double half_wheel_distance = 0.0275;
  double wheel_coordinates[8] = {0.175,  0.1605,  -0.175, 0.1605,
                                 -0.175, -0.1605, 0.175,  -0.1605};  // x1,y1,x2,y2,..,y4
  double pivot_angles_deviation[4] = {-2.5, -1.25, -2.14, 1.49};
  init_kelo_base_config(&kelo_base_config, nWheels, index_to_EtherCAT, radius, castor_offset, half_wheel_distance,
                        wheel_coordinates, pivot_angles_deviation);

  EthercatConfig ethercat_config;
  init_ecx_context(&ethercat_config);

  
  bool debug = true;
  char ifname[] = "eno1";
  int result;

  establish_connection(&ethercat_config, ifname, &result);

  if (result == -1)
  {
    printf("Failed to establish connection\n");
    return 0;
  }

  check_slave_state(&ethercat_config, EC_STATE_SAFE_OP, &result);

  if (result == -1)
  {
    printf("EtherCAT slaves have not reached safe operational state\n");
    return 0;
  }

  process_data_exchange(&ethercat_config, debug);

  check_slave_state(&ethercat_config, EC_STATE_OPERATIONAL, &result);

  if (result == -1)
  {
    printf("EtherCAT slaves have not reached operational state\n");
    return 0;
  }

  const unsigned int N = 3;
  const unsigned int M = 8;

  TorqueControlState torque_control_state;
  init_torque_control_state(&torque_control_state, N, M);

  double platform_force[3] = {0.0, 0.0, 0.0};
  set_platform_force(&torque_control_state, platform_force);

  double pivot_angles[4];
  double wheel_torques[8];

  printf("Reading pivot angles\n");

  read_pivot_angles(&ethercat_config, pivot_angles, index_to_EtherCAT, nWheels);

  // print pivot angles
  for (int i = 0; i < nWheels; i++)
  {
    printf("Pivot angle %d: %f\n", i, pivot_angles[i]);
  }

  set_weight_matrix(&torque_control_state, N, M);

  int counter = 0;
  while (counter < 500)
  {
    usleep(10000);
    compute_wheel_torques(&kelo_base_config, &torque_control_state, pivot_angles,
                          wheel_torques, N, M);
    rxpdo1_t rx_msg;
    create_rx_msg(&rx_msg);
    set_wheel_torques(&ethercat_config, &rx_msg, index_to_EtherCAT, wheel_torques,
                      nWheels, MOTOR_CONST);
    send_and_receive_data(&ethercat_config);
    read_pivot_angles(&ethercat_config, pivot_angles, index_to_EtherCAT, nWheels);
    counter++;
  }

  free_torque_control_state(&torque_control_state);

  return 0;
}