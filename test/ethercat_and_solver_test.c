#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"

int main(int argc, char *argv[])
{
  int nWheels = 4;
  int index_to_EtherCAT[4] = {6, 7, 3, 4};
  KeloBaseConfig kelo_base_config = {0};
  double radius = 0.052;
  double castor_offset = 0.01;
  double half_wheel_distance = 0.0275;
  double wheel_coordinates[8] = {0.175,  0.1605,  -0.175, 0.1605,
                                 -0.175, -0.1605, 0.175,  -0.1605};  // x1,y1,x2,y2,..,y4
  double pivot_angles_deviation[4] = {-2.5, -1.25, -2.14, 1.49};
  init_kelo_base_config(&kelo_base_config, nWheels, index_to_EtherCAT, radius, castor_offset,
                        half_wheel_distance, wheel_coordinates, pivot_angles_deviation);

  EthercatConfig *ethercat_config = calloc(1, sizeof(*ethercat_config));
  init_ecx_context(ethercat_config);

  bool debug = true;
  char ifname[] = "eno1";
  int result;

  establish_connection(ethercat_config, ifname, &result);

  if (result == -1)
  {
    printf("Failed to establish connection\n");
    return 0;
  }

  check_slave_state(ethercat_config, EC_STATE_SAFE_OP, &result);

  if (result == -1)
  {
    printf("EtherCAT slaves have not reached safe operational state\n");
    return 0;
  }

  rxpdo1_t msg;
  memset(&msg, 0, sizeof(msg));
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
    rxpdo1_t *ecData = (rxpdo1_t *)ethercat_config->ecx_slave[index_to_EtherCAT[i]].outputs;
    *ecData = msg;
  }

  process_data_exchange(ethercat_config);

  check_slave_state(ethercat_config, EC_STATE_OPERATIONAL, &result);

  if (result == -1)
  {
    printf("EtherCAT slaves have not reached operational state\n");
    return 0;
  }

  const unsigned int N = 3;
  const unsigned int M = 8;

  TorqueControlState *torque_control_state = calloc(1, sizeof(*torque_control_state));
  init_torque_control_state(torque_control_state, N, M);

  double platform_force[3] = {0.0, 0.0, 30.0};
  set_platform_force(torque_control_state, platform_force, N);

  double pivot_angles[4];
  double wheel_torques[8] = {0.0};

  printf("Reading pivot angles\n");

  double wheel_encoder_values[8] = {0.0};
  double wheel_angular_velocities[8] = {0.0};
  read_encoder_values(ethercat_config, pivot_angles, index_to_EtherCAT, nWheels,
                      kelo_base_config.pivot_angles_deviation, wheel_encoder_values,
                      wheel_angular_velocities);

  // print pivot angles
  for (int i = 0; i < nWheels; i++)
  {
    printf("Pivot angle %d: %f\n", i, pivot_angles[i]);
  }

  set_weight_matrix(torque_control_state, N, M);

  for (size_t i = 0; i < 1; i++)
  {
    wheel_torques[2 * i]     = 1.0;
    wheel_torques[2 * i + 1] = -1.0;
  }

  int counter = 0;
  while (true)
  {
    printf("Counter: %d\n", counter);
    // usleep(10000);
    // compute_wheel_torques(&kelo_base_config, torque_control_state, pivot_angles, wheel_torques, N,
    //                       M);
    rxpdo1_t rx_msg;
    create_rx_msg(&rx_msg);
    set_wheel_torques(ethercat_config, &rx_msg, index_to_EtherCAT, wheel_torques, nWheels,
                      MOTOR_CONST);
    send_and_receive_data(ethercat_config);
    read_encoder_values(ethercat_config, pivot_angles, index_to_EtherCAT, nWheels,
                        kelo_base_config.pivot_angles_deviation, wheel_encoder_values,
                        wheel_angular_velocities);
    for (int i = 0; i < nWheels; i++)
    {
      printf("Pivot angle %d: %f\n", i, pivot_angles[i]);
    }
    counter++;
  }

  free_torque_control_state(torque_control_state);
  free(torque_control_state);
  free(ethercat_config);

  return 0;
}