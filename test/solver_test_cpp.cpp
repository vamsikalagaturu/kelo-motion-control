extern "C"
{
#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"
}

int main(int argc, char *argv[])
{
  KeloBaseConfig kelo_base_config;
  kelo_base_config.nWheels = 4;
  kelo_base_config.index_to_EtherCAT = new int[4]{6, 7, 3, 4};
  kelo_base_config.radius = 0.115 / 2;
  kelo_base_config.castor_offset = 0.01;
  kelo_base_config.half_wheel_distance = 0.0775 / 2;
  kelo_base_config.wheel_coordinates =
      new double[8]{0.188, 0.2075, -0.188, 0.2075, -0.188, -0.2075, 0.188, -0.2075};
  kelo_base_config.pivot_angles_deviation = new double[4]{5.310, 5.533, 1.563, 1.625};

  const unsigned int N = 3;
  const unsigned int M = 8;

  TorqueControlState *torque_control_state = new TorqueControlState();
  init_torque_control_state(torque_control_state, N, M);

  double platform_force[3] = {0.0, 0.0, 10.0};
  set_platform_force(torque_control_state, platform_force, N);

  double pivot_angles[4] = {0.0, 0.0, 0.0, 0.0};
  double wheel_torques[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  set_weight_matrix(torque_control_state, N, M);

  int counter = 0;
  while (counter < 1)
  {
    printf("Counter: %d\n", counter);\
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