/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#include "srs_imu.h"
#include "bmi088.h"
#include "icm20602.h"
#include "icm20689.h"
#include "topics/sensor_imu.h"
#include "sys_time.h"

OS_THREAD                       g_srs_imu_tcb;
UCHAR                           g_srs_imu_stack[SRS_IMU_STACKSIZE];
ULONG                           g_srs_imu_counter = 0;

void srs_imu_entry(ULONG imu_input)
{
	imu_t *imu = sensor_imu();
	sensor_imu_t sensor_imu = {0};
    uorb_advertise(UORB_ID(sensor_imu));

    bmi088_init();

    while (1)
    {
    	bmi088_update_data();

        sensor_imu.gyro[0]   = imu->gyro[0];
    	sensor_imu.gyro[1]   = imu->gyro[1];
    	sensor_imu.gyro[2]   = imu->gyro[2];
    	sensor_imu.accl[0]   = imu->accl[0];
    	sensor_imu.accl[1]   = imu->accl[1];
    	sensor_imu.accl[2]   = imu->accl[2];
        sensor_imu.timestamp = sys_absolute_time();
    	uorb_publish(UORB_ID(sensor_imu), &sensor_imu);

    	OS_TaskDelay(2);
    	g_srs_imu_counter++;
    }
}

