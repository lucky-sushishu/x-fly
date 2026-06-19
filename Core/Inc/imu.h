#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct imu_s imu_t;

struct imu_s
{
    float gyro[3];     /* 角速度 */
    float accl[3];     /* 加速度 */
};

imu_t *sensor_imu(void);

#endif
