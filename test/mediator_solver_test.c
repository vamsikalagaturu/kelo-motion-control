#include "kelo_motion_control/mediator.h"

int main()
{
  KeloBaseConfig *kelo_base_config = calloc(1, sizeof(*kelo_base_config));
  kelo_base_config->nWheels = 4;
  kelo_base_config->index_to_EtherCAT = (int[4]){6, 7, 3, 4};
  kelo_base_config->radius = 0.115 / 2;
  kelo_base_config->castor_offset = 0.01;
  kelo_base_config->half_wheel_distance = 0.0775 / 2;
  kelo_base_config->wheel_coordinates =
      (double[8]){0.188, 0.2075, -0.188, 0.2075, -0.188, -0.2075, 0.188, -0.2075};
  kelo_base_config->pivot_angles_deviation = (double[4]){5.310, 5.533, 1.563, 1.625};

  EthercatConfig *ethercat_config = calloc(1, sizeof(*ethercat_config));
  init_ecx_context(ethercat_config);

  char ifname[] = "eno1";
  int result = 0;

  establish_kelo_base_connection(kelo_base_config, ethercat_config, ifname, &result);

  double pivot_angles[4] = {0.0};
  double wheel_torques[8] = {0.0};

  printf("Reading pivot angles\n");

  double wheel_encoder_values[8] = {0.0};
  double prev_wheel_encoders[8] = {0.0};
  double wheel_angular_velocities[8] = {0.0};

  get_kelo_base_state(kelo_base_config, ethercat_config, pivot_angles, wheel_encoder_values,
                      wheel_angular_velocities);

  const unsigned int N = 3;
  const unsigned int M = 8;

  TorqueControlState *torque_control_state = calloc(1, sizeof(*torque_control_state));
  init_torque_control_state(torque_control_state, N, M);
  set_weight_matrix(torque_control_state, N, M);

  double platform_force[3] = {0.0, 50.0, 0.0};
  set_platform_force(torque_control_state, platform_force, N);

  int counter = 0;
  while (counter < 200)
  {
    printf("Counter: %d\n", counter);
    usleep(10000);
    get_kelo_base_state(kelo_base_config, ethercat_config, pivot_angles, wheel_encoder_values,
                        wheel_angular_velocities);
    compute_wheel_torques(kelo_base_config, torque_control_state, pivot_angles, wheel_torques, N,
                          M);
    set_kelo_base_torques(kelo_base_config, ethercat_config, wheel_torques);
    counter++;
  }

  free(kelo_base_config);
  free(ethercat_config);

  return 0;
}