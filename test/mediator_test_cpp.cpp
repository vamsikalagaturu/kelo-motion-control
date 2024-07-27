#include "kelo_motion_control/mediator.h"

int main()
{
  KeloBaseConfig *kelo_base_config = new KeloBaseConfig();
  kelo_base_config->nWheels = 4;
  kelo_base_config->index_to_EtherCAT = new int[4]{6, 7, 3, 4};
  kelo_base_config->radius = 0.115 / 2;
  kelo_base_config->castor_offset = 0.01;
  kelo_base_config->half_wheel_distance = 0.0775 / 2;
  double wheel_coordinates[8] = {0.188, 0.2075, -0.188, 0.2075, -0.188, -0.2075, 0.188, -0.2075};
  kelo_base_config->wheel_coordinates = wheel_coordinates;
  kelo_base_config->pivot_angles_deviation = new double[4]{5.310, 5.533, 1.563, 1.625};
  
  EthercatConfig *ethercat_config = new EthercatConfig();
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

  for (size_t i = 0; i < 1; i++)
  {
    wheel_torques[2 * i]     = 1.0;
    wheel_torques[2 * i + 1] = -1.0;
  }

  int counter = 0;
  while (true)
  {
    printf("Counter: %d\n", counter);
    usleep(10000);
    set_kelo_base_torques(kelo_base_config, ethercat_config, wheel_torques);
    send_and_receive_data(ethercat_config);
    get_kelo_base_state(kelo_base_config, ethercat_config, pivot_angles, wheel_encoder_values,
                        wheel_angular_velocities);
    counter++;
  }

  free(kelo_base_config);
  free(ethercat_config);

  return 0;
}