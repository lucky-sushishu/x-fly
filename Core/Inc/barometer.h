#ifndef BAROMETER_H
#define BAROMETER_H

#include <stdint.h>

typedef struct barometer_s barometer_t;

struct barometer_s
{
    float pressure;     /* 气压 */
    float temperature;  /* 温度 */
};

barometer_t *sensor_barometer(void);

#endif
