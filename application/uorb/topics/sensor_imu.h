#pragma once
#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include "osapi-uorb.h"


#ifdef __cplusplus
typedef struct  __EXPORT sensor_imu_s {
#else
typedef struct sensor_imu_s {
#endif
	uint64_t timestamp;

    float gyro[3];     /* 角速度 */
    float accl[3];     /* 加速度 */

} sensor_imu_t;

UORB_DECLARE(sensor_imu);

#endif
