#include "magnetometer.h"

#define CONFIG_SENSOR_USE_MAGNETOMETER
#define CONFIG_SENSOR_MAGNETOMETER_TYPE 0


#ifdef CONFIG_SENSOR_USE_MAGNETOMETER
#if CONFIG_SENSOR_MAGNETOMETER_TYPE == 0
#include "ist8310.h"
magnetometer_t *sensor_magnetometer_data = &ist8310.magnetometer;
#else
magnetometer_t *sensor_magnetometer_data = NULL;
#endif
#endif

magnetometer_t *sensor_magnetometer(void)
{
    return sensor_magnetometer_data;
}



