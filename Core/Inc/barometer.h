#ifndef BAROMETER_H
#define BAROMETER_H

#include <stdint.h>

#define BAROMETER_P0 101.325 /* 标准大气压压强，单位kPa */

typedef struct barometer_s barometer_t;

struct barometer_s
{
    float pressure;     /* 气压，单位 mbar */
    float temperature;  /* 温度，单位 ℃ */
};

barometer_t *sensor_barometer(void);

#endif
