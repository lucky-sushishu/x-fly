/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#include "srs_magnetometer.h"
#include "ist8310.h"
#include "topics/sensor_magnetometer.h"
#include "sys_time.h"

OS_THREAD                       g_srs_magnetometer_tcb;
UCHAR                           g_srs_magnetometer_stack[SRS_MAGNETOMETER_STACKSIZE];
ULONG                           g_srs_magnetometer_counter = 0;

void srs_magnetometer_entry(ULONG magnetometer_input)
{
	magnetometer_t *magnetometer = sensor_magnetometer();
	sensor_magnetometer_t sensor_magnetometer = {0};
    uorb_advertise(UORB_ID(sensor_magnetometer));

    ist8310_init();

    while (1)
    {
    	ist8310_update_data();

        sensor_magnetometer.mag[0] 	  = magnetometer->mag[0];
    	sensor_magnetometer.mag[1] 	  = magnetometer->mag[1];
    	sensor_magnetometer.mag[2] 	  = magnetometer->mag[2];
		sensor_magnetometer.timestamp = sys_absolute_time();
    	uorb_publish(UORB_ID(sensor_magnetometer), &sensor_magnetometer);

    	OS_TaskDelay(50);
    	g_srs_magnetometer_counter++;
    }
}

