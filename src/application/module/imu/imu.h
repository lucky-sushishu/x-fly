#ifndef _IMU_H_
#define _IMU_H_

#include "includes.h"
#include "tx_api.h"

#include "mpu9250.h"
#include "math.h"
#include "MahonyAHRS.h"

#include "ano.h"

#define G (9.81f)
#define MAG (50.0f)

#define IMU_MAG_PRIO 14
#define IMU_MAG_STACKSIZE 2048
#define IMU_QUEUE_SIZE 50

extern TX_THREAD imu_mag_tcb;
extern UCHAR imu_mag_stack[IMU_MAG_STACKSIZE];
extern TX_QUEUE queue_imu;
extern UCHAR queue_imu_area[3*sizeof(float)*IMU_QUEUE_SIZE];
extern TX_EVENT_FLAGS_GROUP event_flags_led;

typedef struct sensor_imu_s
{
  float gyro[3];
  float acce[3];
} sensor_imu_t;

typedef struct sensor_mag_s
{
  float data[3];
} sensor_mag_t;

typedef struct euler_rad_s {
  float roll;
  float pitch;
  float yaw;
} euler_rad_t;

typedef struct quaternion_s {
  float v0;
  float v1;
  float v2;
  float v3;
} quaternion_t;

void imu_mag_entry(ULONG thread_input);

#endif // _IMU_H_
