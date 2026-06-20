#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct imu_s imu_t;

struct imu_s
{
    float gyro[3];     /* 角速度，单位：°/s */
    float accl[3];     /* 加速度，单位：m^2/s  */
};

imu_t *sensor_imu(void);

#endif
