#pragma once
#ifndef SENSOR_MAGNETOMETER_H
#define SENSOR_MAGNETOMETER_H

#include "osapi-uorb.h"


#ifdef __cplusplus
typedef struct  __EXPORT sensor_magnetometer_s {
#else
typedef struct sensor_magnetometer_s {
#endif
	uint64_t timestamp;
	float mag[3];     	/* 磁场强度 */
} sensor_magnetometer_t;

UORB_DECLARE(sensor_magnetometer);

#endif
