#ifndef __IMU_H
#define __IMU_H

#include "tx_api.h"

#define IMU_MAG_PRIO 14
#define IMU_MAG_STACKSIZE 1024
extern TX_THREAD imu_mag_tcb;
extern UCHAR imu_mag_stack[IMU_MAG_STACKSIZE];

typedef struct sensor_imu_s
{
  float gyro[3];
  float acce[3];
} sensor_imu_t;

typedef struct sensor_mag_s
{
  float data[3];
} sensor_mag_t;

void imu_mag_entry(ULONG thread_input);

#endif // __IMU_H
