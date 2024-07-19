#include "kelo_motion_control/mediator.h"

void establish_kelo_base_connection(KeloBaseConfig* kelo_base_config,
                                    EthercatConfig* ethercat_config, char* ifname, int* result)
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
  memset(&msg, 0, sizeof(msg));
  msg.timestamp = 1;
  msg.command1 = 0;
  msg.command2 = 0;
  msg.limit1_p = 0;
  msg.limit1_n = 0;
  msg.limit2_p = 0;
  msg.limit2_n = 0;
  msg.setpoint1 = 0;
  msg.setpoint2 = 0;

  for (size_t i=0; i < kelo_base_config->nWheels; i++)
  {
    printf("index_to_EtherCAT[%ld]: %d\n", i, kelo_base_config->index_to_EtherCAT[i]);
  }

  for (size_t i = 0; i < kelo_base_config->nWheels; i++)
  {
    rxpdo1_t *ecData = (rxpdo1_t *)ethercat_config->ecx_slave[kelo_base_config->index_to_EtherCAT[i]].outputs;
    memcpy(ecData, &msg, sizeof(msg));
  }

  process_data_exchange(ethercat_config);
  check_slave_state(ethercat_config, EC_STATE_OPERATIONAL, result);
  if (*result == -1)
  {
    printf("EtherCAT slaves have not reached operational state\n");
    return;
  }
}

void update_base_state(KeloBaseConfig* kelo_base_config, EthercatConfig* ethercat_config)
{
  rxpdo1_t rx_msg;
  memset(&rx_msg, 0, sizeof(rx_msg));
  create_rx_msg(&rx_msg);
  for (size_t i = 0; i < kelo_base_config->nWheels; i++)
  {
    rxpdo1_t *ecData = (rxpdo1_t *)ethercat_config->ecx_slave[kelo_base_config->index_to_EtherCAT[i]].outputs;
    memcpy(ecData, &rx_msg, sizeof(rx_msg));
  }
  send_and_receive_data(ethercat_config);
}

void get_kelo_base_state(KeloBaseConfig* kelo_base_config, EthercatConfig* ethercat_config,
                         double* pivot_angles, double* wheel_encoder_values,
                         double *wheel_angular_velocities)
{
  read_encoder_values(ethercat_config, pivot_angles, kelo_base_config->index_to_EtherCAT,
                      kelo_base_config->nWheels, kelo_base_config->pivot_angles_deviation,
                      wheel_encoder_values, wheel_angular_velocities);
}

void get_kelo_wheel_voltages_and_currents(KeloBaseConfig* kelo_base_config,
                                          EthercatConfig* ethercat_config, double* wheel_voltages,
                                          double* wheel_currents)
{
  read_voltages_and_currents(ethercat_config, kelo_base_config->index_to_EtherCAT,
                                   kelo_base_config->nWheels, wheel_voltages, wheel_currents);
}

void set_kelo_base_torques(KeloBaseConfig* kelo_base_config, EthercatConfig* ethercat_config,
                           double* wheel_torques)
{
  rxpdo1_t rx_msg;
  memset(&rx_msg, 0, sizeof(rx_msg));
  create_rx_msg(&rx_msg);
  set_wheel_torques(ethercat_config, &rx_msg, kelo_base_config->index_to_EtherCAT, wheel_torques,
                    kelo_base_config->nWheels, MOTOR_CONST);
  send_and_receive_data(ethercat_config);
}

void calculate_robot_velocity(double* vx, double* vy, double* va, double* encDisplacement,
                              double* prev_wheel_encoders, double* pivot_angles,
                              double* wheel_encoders, KeloBaseConfig* kelo_base_config)
{
  double dt = 0.1;

  // initialize the variables
  *vx = 0;
  *vy = 0;
  *va = 0;

  for (int i = 0; i < kelo_base_config->nWheels; i++)
  {
    double wl = (wheel_encoders[2 * i] - prev_wheel_encoders[2 * i]) / dt;
    double wr = -(wheel_encoders[2 * i + 1] - prev_wheel_encoders[2 * i + 1]) / dt;

    prev_wheel_encoders[2 * i] = wheel_encoders[2 * i];
    prev_wheel_encoders[2 * i + 1] = wheel_encoders[2 * i + 1];

    double theta = pivot_angles[i];

    *vx -= kelo_base_config->radius *
           ((wl + wr) * cos(theta));  // + 2 * s_d_ratio * (wl - wr) * sin(theta));
    *vy -= kelo_base_config->radius *
           ((wl + wr) * sin(theta));  // - 2 * s_d_ratio * (wl - wr) * cos(theta));

    double wangle = atan2(kelo_base_config->wheel_coordinates[2 * i + 1],
                          kelo_base_config->wheel_coordinates[2 * i]);
    double d = sqrt(kelo_base_config->wheel_coordinates[2 * i] *
                        kelo_base_config->wheel_coordinates[2 * i] +
                    kelo_base_config->wheel_coordinates[2 * i + 1] *
                        kelo_base_config->wheel_coordinates[2 * i + 1]);

    double s_d_ratio =
        kelo_base_config->castor_offset / (kelo_base_config->half_wheel_distance * 2);

    *va += kelo_base_config->radius *
           (2 * (wr - wl) * s_d_ratio * cos(theta - wangle) - (wr + wl) * sin(theta - wangle)) / d;

    // va += r_w * (wr + wl) * sin(theta - wangle) / d;
    // va += 4*swData->gyro_y;
  }
  // averaging the wheel velocity
  *vx = *vx / kelo_base_config->nWheels / 2;
  *vy = *vy / kelo_base_config->nWheels / 2;
  *va = *va / kelo_base_config->nWheels / 2;
}

void calculate_robot_pose(double vx, double vy, double va, double* odomx, double* odomy,
                          double* odoma)
{
  double dt = 0.1;
  double dx, dy;

  if (fabs(va) > 0.001)
  {
    double vlin = sqrt(vx * vx + vy * vy);
    double direction = atan2(vy, vx);
    double circleRadius = fabs(vlin / va);
    double sign = 1;
    if (va < 0)
      sign = -1;
    // displacement relative to direction of movement
    double dx_rel = circleRadius * sin(fabs(va) * dt);
    double dy_rel = sign * circleRadius * (1 - cos(fabs(va) * dt));

    // transform displacement to previous robot frame
    dx = dx_rel * cos(direction) - dy_rel * sin(direction);
    dy = dx_rel * sin(direction) + dy_rel * cos(direction);
  }
  else
  {
    dx = vx * dt;
    dy = vy * dt;
  }

  // transform displacement to odom frame
  *odomx += dx * cos(*odoma) - dy * sin(*odoma);
  *odomy += dx * sin(*odoma) + dy * cos(*odoma);
  *odoma = norm(*odoma + va * dt);
}

double norm(double x)
{
  const double TWO_PI = 2.0 * M_PI;
  while (x < -M_PI)
  {
    x += TWO_PI;
  }
  while (x > M_PI)
  {
    x -= TWO_PI;
  }

  return x;
}

void update_odom(double* odomx, double* odomy, double* odoma, double* wheel_encoders,
                 double* prev_wheel_encoders, double* pivot_angles,
                 KeloBaseConfig* kelo_base_config)
{
  double vx, vy, va, encDisplacement;
  calculate_robot_velocity(&vx, &vy, &va, &encDisplacement, prev_wheel_encoders, pivot_angles,
                           wheel_encoders, kelo_base_config);
  calculate_robot_pose(vx, vy, va, odomx, odomy, odoma);
}