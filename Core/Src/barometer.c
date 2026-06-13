#include "barometer.h"

#define CONFIG_SENSOR_USE_BAROMETER
#define CONFIG_SENSOR_BAROMETER_TYPE 0


#ifdef CONFIG_SENSOR_USE_BAROMETER
#if CONFIG_SENSOR_BAROMETER_TYPE == 0
#include "ms5611.h"
barometer_t *sensor_barometer_data = &ms5611.barometer;
#else
barometer_t *sensor_barometer_data = NULL;
#endif
#endif

barometer_t *sensor_barometer(void)
{
    return sensor_barometer_data;
}



