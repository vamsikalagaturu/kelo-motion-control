#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"

int main(int argc, char *argv[])
{
  int nWheels = 4;
  int index_to_EtherCAT[4] = {3, 5, 7, 9};
  double radius = 0.052;
  double castor_offset = 0.01;
  double half_wheel_distance = 0.0275;
  double wheel_coordinates[8] = {0.175,  0.1605,  -0.175, 0.1605,
                                 -0.175, -0.1605, 0.175,  -0.1605};  // x1,y1,x2,y2,..,y4
  double pivot_angles_deviation[4] = {-2.5, -1.25, -2.14, 1.49};
  KeloBaseConfig kelo_base_config;
  init_kelo_base_config(&kelo_base_config, nWheels, index_to_EtherCAT, radius, castor_offset, half_wheel_distance,
                        wheel_coordinates, pivot_angles_deviation);

  const unsigned int N = 3;
  const unsigned int M = 8;

  TorqueControlState *torque_control_state = malloc(sizeof(TorqueControlState));
  init_torque_control_state(torque_control_state, N, M);

  double platform_force[3] = {0.0, 0.0, 10.0};
  set_platform_force(torque_control_state, platform_force, N);

  double pivot_angles[4] = {0.0, 0.0, 0.0, 0.0};
  double wheel_torques[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  set_weight_matrix(torque_control_state, N, M);

  int counter = 0;
  while (counter < 1)
  {
    printf("Counter: %d\n", counter);
    usleep(10000);
    compute_wheel_torques(&kelo_base_config, torque_control_state, pivot_angles,
                          wheel_torques, N, M);
    printf("Wheel torques: ");
    for (size_t i = 0; i < M; i++)
    {
      printf("%f ", wheel_torques[i]);
    }
    printf("\n");
    counter++;
  }

  free_torque_control_state(torque_control_state);
  free(torque_control_state);

  return 0;
}