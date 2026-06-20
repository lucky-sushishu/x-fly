/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#include "srs_barometer.h"
#include "ms5611.h"
#include "topics/sensor_barometer.h"
#include "sys_time.h"

OS_THREAD                       	g_srs_barometer_tcb;
UCHAR                           	g_srs_barometer_stack[SRS_BAROMETER_STACKSIZE];
ULONG                           	g_srs_barometer_counter = 0;

void srs_barometer_entry(ULONG barometer_input)
{
	barometer_t *barometer = sensor_barometer();
	sensor_barometer_t sensor_barometer = {0};
    uorb_advertise(UORB_ID(sensor_barometer));

    while (1)
    {
    	bmi088_update_data();

        sensor_barometer.pressure    = barometer->pressure;
		sensor_barometer.temperature = barometer->temperature;
		sensor_barometer.timestamp   = sys_absolute_time();
    	uorb_publish(UORB_ID(sensor_barometer), &sensor_barometer);

    	OS_TaskDelay(50);
    	g_srs_barometer_counter++;
    }
}

