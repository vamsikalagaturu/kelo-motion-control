#ifndef KELO_MOTION_CONTROL_MEDIATOR_H
#define KELO_MOTION_CONTROL_MEDIATOR_H

#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"

// Initialization Functions
void initialize_kelo_base(KeloBaseConfig* kelo_base_config, EthercatConfig* ethercat_config);

void establish_kelo_base_connection(KeloBaseConfig* kelo_base_config,
                                    EthercatConfig* ethercat_config, char* ifname, int* result);

void get_kelo_base_state(KeloBaseConfig* kelo_base_config, EthercatConfig* ethercat_config,
                         double* pivot_angles, double* wheel_encoder_values,
                         double *wheel_angular_velocities);

void set_kelo_base_torques(KeloBaseConfig* kelo_base_config, EthercatConfig* ethercat_config,
                           double* wheel_torques);

void update_base_state(KeloBaseConfig* kelo_base_config, EthercatConfig* ethercat_config);

void calculate_robot_velocity(double* vx, double* vy, double* va, double* encDisplacement,
                              double* prev_wheel_encoders, double* pivot_angles,
                              double* wheel_encoders, KeloBaseConfig* kelo_base_config);

void calculate_robot_pose(double vx, double vy, double va, double* odomx, double* odomy,
                          double* odoma);

double norm(double x);

#endif  // KELO_MOTION_CONTROL_MEDIATOR_H