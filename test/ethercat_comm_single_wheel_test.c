#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"

int main(int argc, char *argv[])
{
  int nWheels = 1;
  int index_to_EtherCAT[1] = {4};
  double radius = 0.052;
  double castor_offset = 0.01;
  double half_wheel_distance = 0.0275;
  double wheel_coordinates[2] = {0.175, 0.1605};  // x1,y1,x2,y2,..,y4
  double pivot_angles_deviation[1] = {-2.5};

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
    rxpdo1_t *ecData =
        (rxpdo1_t *)ethercat_config->ecx_slave[index_to_EtherCAT[i]].outputs;
    *ecData = msg;
  }

  process_data_exchange(ethercat_config);

  check_slave_state(ethercat_config, EC_STATE_OPERATIONAL, &result);

  if (result == -1)
  {
    printf("EtherCAT slaves have not reached operational state\n");
    return 0;
  }

  double pivot_angles[1];
  double wheel_torques[2] = {0.0, 0.0};

  printf("Reading pivot angles\n");

  read_pivot_angles(ethercat_config, pivot_angles, index_to_EtherCAT, nWheels,
                    pivot_angles_deviation);

  // print pivot angles
  for (int i = 0; i < nWheels; i++)
  {
    printf("Pivot angle %d: %f\n", i, pivot_angles[i]);
  }

  int counter = 0;
  while (counter < 5)
  {
    printf("Counter: %d\n", counter);
    usleep(100000);
    ecx_receive_processdata(&ethercat_config->ecx_context, EC_TIMEOUTRET);
    for (unsigned int i = 0; i < nWheels; i++)
    {
      txpdo1_t *ecData =
          (txpdo1_t *)ethercat_config->ecx_slave[index_to_EtherCAT[i]].inputs;
      pivot_angles[i] = ecData->encoder_pivot;
      printf("Pivot angle %d: %f\n", i, pivot_angles[i]);
    }

    rxpdo1_t rx_msg;
    memset(&rx_msg, 0, sizeof(rx_msg));
    create_rx_msg(&rx_msg);
    set_wheel_torques(ethercat_config, &rx_msg, index_to_EtherCAT, wheel_torques, nWheels,
                      MOTOR_CONST);
    ecx_send_processdata(&ethercat_config->ecx_context);

    counter++;
  }

  free(ethercat_config);

  return 0;
}