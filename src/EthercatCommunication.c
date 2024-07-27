/**
 * @file EthercatCommunication.c
 * @author Kavya Shankar (kavya.shankar@smail.inf.h-brs.de)
 * @brief Establishing connection with ehtercat and performing data transfer with robot
 * @date 2022-03-12
 *
 */
#include "kelo_motion_control/EthercatCommunication.h"
#include <math.h>
#define _USE_MATH_DEFINES

void init_ecx_context(EthercatConfig *config)
{
  // Initialize context pointers to the relevant parts of the configuration structure.
  config->ecx_context.port              = &config->ecx_port;
  config->ecx_context.slavelist         = &config->ecx_slave[0];
  config->ecx_context.slavecount        = &config->ecx_slavecount;
  config->ecx_context.maxslave          = EC_MAXSLAVE;
  config->ecx_context.grouplist         = &config->ec_group[0];
  config->ecx_context.maxgroup          = EC_MAXGROUP;
  config->ecx_context.esibuf            = &config->esibuf[0];
  config->ecx_context.esimap            = &config->esimap[0];
  config->ecx_context.esislave          = 0;  // Assuming initial value as 0
  config->ecx_context.elist             = &config->ec_elist;
  config->ecx_context.idxstack          = &config->ec_idxstack;

  config->ecx_context.ecaterror         = &config->EcatError;
  config->ecx_context.DCtime            = &config->ec_DCtime;
  config->ecx_context.SMcommtype        = &config->ec_SMcommtype;
  config->ecx_context.PDOassign         = &config->ec_PDOassign;
  config->ecx_context.PDOdesc           = &config->ec_PDOdesc;
  config->ecx_context.eepSM             = &config->ec_SM;
  config->ecx_context.eepFMMU           = &config->ec_FMMU;
  config->ecx_context.manualstatechange = 0;  // 0 typically means no manual state change allowed
}

void establish_connection(EthercatConfig *config, char *ifname, int *result)
{
  if (!ecx_init(&config->ecx_context, ifname))
  {
    printf("Failed to initialize EtherCAT\n");
    *result = -1;
  }
  int wkc = ecx_config_init(&config->ecx_context, TRUE);
  if (wkc <= 0)
  {
    printf("No slaves found\n");
    *result = -1;
  }
  else
  {
    printf("Found %d slaves\n", wkc);
    *result = 0;
  }
  int iomap_size = ecx_config_map_group(&config->ecx_context, config->IOmap, 0);
  printf("IOmap size: %d\n", iomap_size);
  *result = 0;
}

void check_slave_state(EthercatConfig *config, uint16 required_state, int *result)
{
  ecx_statecheck(&config->ecx_context, 0, required_state, EC_TIMEOUTSTATE);

  if (config->ecx_slave[0].state != required_state)
  {
    ecx_readstate(&config->ecx_context);
    for (int i = 1; i <= config->ecx_slavecount; i++)
    {
      if (config->ecx_slave[i].state != required_state)
      {
        printf("Slave %i State= %i\n", i, config->ecx_slave[i].state);
      }
    }
    *result = -1;
  }
  *result = 0;
}

void process_data_exchange(EthercatConfig *config)
{
  ecx_send_processdata(&config->ecx_context);
  ecx_receive_processdata(&config->ecx_context, EC_TIMEOUTRET);
  config->ecx_slave[0].state = EC_STATE_OPERATIONAL;
  // ecx_send_processdata(&config->ecx_context);
  ecx_writestate(&config->ecx_context, 0);
}

void send_and_receive_data(EthercatConfig *config)
{
  ecx_send_processdata(&config->ecx_context);
  ecx_receive_processdata(&config->ecx_context, EC_TIMEOUTRET);
}

void create_rx_msg(rxpdo1_t *msg)
{
  msg->timestamp = time(NULL);
  msg->command1  = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_TORQUE;
  msg->limit1_p  = 0.0;  // upper limit for first wheel
  msg->limit1_n  = 0.0;  // lower limit for first wheel
  msg->limit2_p  = 0.0;  // upper limit for second wheel
  msg->limit2_n  = 0.0;  // lower limit for second wheel
  msg->setpoint1 = 0.0;  // setpoint for first wheel
  msg->setpoint2 = 0.0;  // setpoint for second wheel
}

void set_wheel_torques(EthercatConfig *config, rxpdo1_t *msg, int *index_to_EtherCAT,
                       double *wheel_torques, int nWheels, double motor_const)
{
  for (unsigned int i = 0; i < nWheels; i++)
  {
    // Update torque values in the message for each wheel
    // msg->setpoint1 = -motor_const * wheel_torques[2 * i];     // units: (rad/sec) for first wheel
    // msg->setpoint2 = motor_const * wheel_torques[2 * i + 1];  // units: (rad/sec) for second wheel

    // units: (rad/sec)
    msg->setpoint1 = -wheel_torques[2 * i] / motor_const;   // negative due to inverted frame 
    msg->setpoint2 = wheel_torques[2 * i + 1] / motor_const;

    // Get the output pointer for the current wheel based on EtherCAT mapping
    rxpdo1_t *ecData = (rxpdo1_t *)config->ecx_slave[index_to_EtherCAT[i]].outputs;

    // Copy the updated message to the slave's output
    memcpy(ecData, msg, sizeof(rxpdo1_t));
  }
}

void read_encoder_values(EthercatConfig *config, double *pivot_angles, int *index_to_EtherCAT,
                         int nWheels, double *pivot_angles_deviation, double *wheel_encoder_values,
                         double *wheel_angular_velocities)
{
  for (unsigned int i = 0; i < nWheels; i++)
  {
    txpdo1_t *ecData = (txpdo1_t *)config->ecx_slave[index_to_EtherCAT[i]].inputs;
    pivot_angles[i] = ecData->encoder_pivot - pivot_angles_deviation[i];
    if (pivot_angles[i] > 2 * M_PI)
      pivot_angles[i] -= 2 * M_PI;
    else if (pivot_angles[i] < 0.0)
      pivot_angles[i] += 2 * M_PI;

    wheel_encoder_values[2 * i] = ecData->encoder_1;
    wheel_encoder_values[2 * i + 1] = ecData->encoder_2;

    wheel_angular_velocities[2 * i] = ecData->velocity_1;
    wheel_angular_velocities[2 * i + 1] = ecData->velocity_2;
  }
}

void read_voltages_and_currents(EthercatConfig *config, int *index_to_EtherCAT, int nWheels,
                                double *bus_voltages, double *wheel_voltages,
                                double *wheel_currents)
{
  for (unsigned int i = 0; i < nWheels; i++)
  {
    txpdo1_t *ecData = (txpdo1_t *)config->ecx_slave[index_to_EtherCAT[i]].inputs;
    wheel_voltages[2 * i] = ecData->voltage_1;
    wheel_voltages[2 * i + 1] = ecData->voltage_2;

    bus_voltages[i] = ecData->voltage_bus;

    double u1 = ecData->current_1_u;
    double v1 = ecData->current_1_v;
    double w1 = ecData->current_1_w;
    wheel_currents[2 * i] = sqrt((u1 * u1 + v1 * v1 + w1 * w1) / 3.0);

    double u2 = ecData->current_2_u;
    double v2 = ecData->current_2_v;
    double w2 = ecData->current_2_w;
    wheel_currents[2 * i + 1] = sqrt((u2 * u2 + v2 * v2 + w2 * w2) / 3.0);
  }
}