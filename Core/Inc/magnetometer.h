#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <stdint.h>

typedef struct magnetometer_s magnetometer_t;

struct magnetometer_s
{
    float mag[3];     /* 磁场强度 */
};

magnetometer_t *sensor_magnetometer(void);

#endif
