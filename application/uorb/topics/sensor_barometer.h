#pragma once
#ifndef SENSOR_BAROMETER_H
#define SENSOR_BAROMETER_H

#include "osapi-uorb.h"


#ifdef __cplusplus
typedef struct  __EXPORT sensor_barometer_s {
#else
typedef struct sensor_barometer_s {
#endif
	uint64_t timestamp;
	float pressure;     /* 气压，单位 mbar */
    float temperature;  /* 温度，单位 ℃ */

} sensor_barometer_t;

UORB_DECLARE(sensor_barometer);

#endif
