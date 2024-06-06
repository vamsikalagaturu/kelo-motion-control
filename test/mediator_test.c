#include "kelo_motion_control/mediator.h"

int main()
{
  int nWheels = 4;
  int index_to_EtherCAT[4] = {6, 7, 3, 4};
  double radius = 0.052;
  double castor_offset = 0.01;
  double half_wheel_distance = 0.0275;
  double wheel_coordinates[8] = {0.175,  0.1605,  -0.175, 0.1605,
                                 -0.175, -0.1605, 0.175,  -0.1605};  // x1,y1,x2,y2,..,y4
  double pivot_angles_deviation[4] = {5.310, 5.533, 1.563, 1.625};

  KeloBaseConfig *kelo_base_config = calloc(1, sizeof(*kelo_base_config));
  EthercatConfig *ethercat_config = calloc(1, sizeof(*ethercat_config));

  init_kelo_base_config(kelo_base_config, nWheels, index_to_EtherCAT, radius, castor_offset,
                        half_wheel_distance, wheel_coordinates, pivot_angles_deviation);

  init_ecx_context(ethercat_config);

  char ifname[] = "eno1";
  int result = 0;

  establish_kelo_base_connection(kelo_base_config, ethercat_config, ifname, &result);

  double pivot_angles[4] = {0.0};
  double wheel_torques[8] = {0.0};

  printf("Reading pivot angles\n");

  double wheel_encoder_values[8] = {0.0};
  double prev_wheel_encoders[8] = {0.0};
  double odomx = 0.0;
  double odomy = 0.0;
  double odoma = 0.0;

  get_kelo_base_state(kelo_base_config, ethercat_config, pivot_angles, wheel_encoder_values,
                      prev_wheel_encoders, &odomx, &odomy, &odoma);

  int counter = 0;
  while (true)
  {
    printf("Counter: %d\n", counter);
    usleep(10000);
    send_and_receive_data(ethercat_config);
    get_kelo_base_state(kelo_base_config, ethercat_config, pivot_angles, wheel_encoder_values,
                      prev_wheel_encoders, &odomx, &odomy, &odoma);
    
    printf("odom: %f, %f, %f\n", odomx, odomy, odoma);

    counter++;
  }

  free(kelo_base_config);
  free(ethercat_config);

  return 0;
}