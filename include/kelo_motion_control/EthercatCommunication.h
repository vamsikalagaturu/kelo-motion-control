#ifndef KELO_MOTION_CONTROL_ETHERCAT_COMMUNICATION_H
#define KELO_MOTION_CONTROL_ETHERCAT_COMMUNICATION_H

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "soem/ethercat.h"
#include "soem/ethercattype.h"
#include "soem/nicdrv.h"
#include "soem/ethercatbase.h"
#include "soem/ethercatmain.h"
#include "soem/ethercatconfig.h"
#include "soem/ethercatcoe.h"
#include "soem/ethercatdc.h"
#include "soem/ethercatprint.h"
#include "kelo_motion_control/KeloDriveAPI.h"

#define IO_MAP_SIZE 4096

typedef struct
{
  ec_slavet ecx_slave[EC_MAXSLAVE];
  int ecx_slavecount;
  ec_groupt ec_group[EC_MAXGROUP];
  uint8 esibuf[EC_MAXEEPBUF];
  uint32 esimap[EC_MAXEEPBITMAP];
  ec_eringt ec_elist;
  ec_idxstackT ec_idxstack;
  ec_SMcommtypet ec_SMcommtype;
  ec_PDOassignt ec_PDOassign;
  ec_PDOdesct ec_PDOdesc;
  ec_eepromSMt ec_SM;
  ec_eepromFMMUt ec_FMMU;
  boolean EcatError;
  int64 ec_DCtime;
  ecx_portt ecx_port;
  ecx_redportt ecx_redport;
  ecx_contextt ecx_context;
  char IOmap[IO_MAP_SIZE];
} EthercatConfig;

void init_ecx_context(EthercatConfig *config);

void establish_connection(EthercatConfig *config, char *ifname, int *result);

void check_slave_state(EthercatConfig *config, uint16 required_state, int *result);

void process_data_exchange(EthercatConfig *config);

void send_and_receive_data(EthercatConfig *config);

void create_rx_msg(rxpdo1_t *msg);

void set_wheel_torques(EthercatConfig *config, rxpdo1_t *msg, int *index_to_EtherCAT,
                       double *wheel_torques, int nWheels, double motor_const);

void read_pivot_angles(EthercatConfig *config, double *pivot_angles, int *index_to_EtherCAT,
                       int nWheels, double *pivot_angles_deviation);

#endif  // KELO_MOTION_CONTROL_ETHERCAT_COMMUNICATION_H