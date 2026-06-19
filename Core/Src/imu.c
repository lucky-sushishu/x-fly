#include "imu.h"

#define CONFIG_SENSOR_USE_IMU
#define CONFIG_SENSOR_IMU_TYPE 0


#ifdef CONFIG_SENSOR_USE_IMU
#if CONFIG_SENSOR_IMU_TYPE == 0 // bmi088
#include "bmi088.h"
imu_t *sensor_imu_data = &bmi088.imu;
#elif CONFIG_SENSOR_IMU_TYPE == 1 // icm20602
#include "icm20602.h"
imu_t *sensor_imu_data = &icm20602.imu;
#elif CONFIG_SENSOR_IMU_TYPE == 2 // icm20689
#include "icm20689.h"
imu_t *sensor_imu_data = &icm20689.imu;
#else
imu_t *sensor_imu_data = NULL;
#endif
#endif

imu_t *sensor_imu(void)
{
    return sensor_imu_data;
}



