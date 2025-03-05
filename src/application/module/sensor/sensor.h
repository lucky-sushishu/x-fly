#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "includes.h"
#include "tx_api.h"

#include "mpu9250.h"
#include "math.h"
// #include "MahonyAHRS.h"
#include "MadgwickAHRS.h"

#include "ano.h"

#define SENSOR_PRIO 14
#define SENSOR_STACKSIZE (4 * 1024)

extern TX_THREAD sensor_tcb;
extern UCHAR sensor_stack[SENSOR_STACKSIZE];

typedef struct mpu9250_imu_data_s {
  int8_t accl_xout_h;
  uint8_t accl_xout_l;
  int8_t accl_yout_h;
  uint8_t accl_yout_l;
  int8_t accl_zout_h;
  uint8_t accl_zout_l;
  int8_t temp_out_h;
  uint8_t temp_out_l;
  int8_t gyro_xout_h;
  uint8_t gyro_xout_l;
  int8_t gyro_yout_h;
  uint8_t gyro_yout_l;
  int8_t gyro_zout_h;
  uint8_t gyro_zout_l;
} mpu9250_imu_data_t;

typedef struct sensor_imu_s
{
  float accl[3];
  float gyro[3];
} sensor_imu_t;

typedef struct sensor_mag_s
{
  float data[3];
} sensor_mag_t;

/* Communication data */
typedef struct imu_s {
  float accl[3];
  float gyro[3];
} imu_t;
typedef struct mag_s {
  float data[3];
} mag_t;
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

typedef struct communication_data_s {
  imu_t imu;
  mag_t mag;
  euler_rad_t euler_rad;
  quaternion_t quaternion;
} communication_data_t;

void sensor_entry(ULONG thread_input);

#endif // _SENSOR_H_
