#ifndef KELO_MOTION_CONTROL_MEDIATOR_H
#define KELO_MOTION_CONTROL_MEDIATOR_H

#include "kelo_motion_control/EthercatCommunication.h"
#include "kelo_motion_control/KeloMotionControl.h"

// Initialization Functions
void initialize_kelo_base(KeloBaseConfig* kelo_base_config,
                          EthercatConfig* ethercat_config);

void establish_kelo_base_connection(KeloBaseConfig* kelo_base_config, EthercatConfig* ethercat_config, char* ifname,
                                    int* result);

void get_kelo_base_state(KeloBaseConfig* kelo_base_config,
                         EthercatConfig* ethercat_config, double* pivot_angles);

void set_kelo_base_torques(KeloBaseConfig* kelo_base_config,
                           EthercatConfig* ethercat_config, double* wheel_torques);

#endif  // KELO_MOTION_CONTROL_MEDIATOR_H