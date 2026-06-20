/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#include "srs_attitude.h"
#include "sys_time.h"

#include "topics/sensor_imu.h"
#include "topics/sensor_barometer.h"
#include "topics/sensor_magnetometer.h"
#include "MadgwickAHRS.h"
#include "usart.h"
#include "ano.h"

OS_THREAD                       	g_srs_attitude_tcb;
UCHAR                           	g_srs_attitude_stack[SRS_ATTITUDE_STACKSIZE];
ULONG                           	g_srs_attitude_counter = 0;

#define GYRO_BAIS_SAMPLE_NUM        500

void q2euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw);

void srs_attitude_entry(ULONG attitude_input)
{
    status_t status                           = INIT;
    attitude_t attitude                       = {0.0f};

    bool sensor_imu_update                    = false;
    bool sensor_barometer_update              = false;
    bool sensor_magnetometer_update           = false;
    sensor_imu_t sensor_imu                   = {0};
    sensor_barometer_t sensor_barometer       = {0};
    sensor_magnetometer_t sensor_magnetometer = {0};
    uorb_sub_t imu_sub                        = uorb_subscribe(UORB_ID(sensor_imu));
    uorb_sub_t barometer_sub                  = uorb_subscribe(UORB_ID(sensor_barometer));
    uorb_sub_t magnetometer_sub               = uorb_subscribe(UORB_ID(sensor_magnetometer));
    
    int i                                     = GYRO_BAIS_SAMPLE_NUM;
    float gyro_bais[3]                        = {0.0f};

    uint8_t *data                             = NULL;
    attitude_euler_t attitude_euler           = {0};
    attitude_quaternion_t attitude_quaternion = {0};

    while (1)
    {
        switch (status)
        {
            case INIT : {
                            uorb_check(UORB_ID(sensor_imu), imu_sub, &sensor_imu_update);
                            if (sensor_imu_update) {
                                uorb_copy(UORB_ID(sensor_imu), &imu_sub, &sensor_imu);
                                if (i-- > 0) {
                                    gyro_bais[0] += sensor_imu.gyro[0]; gyro_bais[1] += sensor_imu.gyro[1]; gyro_bais[2] += sensor_imu.gyro[2];
                                }
                                else {
                                    gyro_bais[0] = gyro_bais[0] / GYRO_BAIS_SAMPLE_NUM; gyro_bais[1] = gyro_bais[1] / GYRO_BAIS_SAMPLE_NUM; gyro_bais[2] = gyro_bais[2] / GYRO_BAIS_SAMPLE_NUM;
                                    status       = WORK;
                                }
                            }
                            OS_TaskDelay(1);
                            break;
                        }
            case WORK : {
                            uorb_check(UORB_ID(sensor_imu), imu_sub, &sensor_imu_update);
                            if (sensor_imu_update) {
                                uorb_copy(UORB_ID(sensor_imu), &imu_sub, &sensor_imu);
                                attitude.gx = (sensor_imu.gyro[0] - gyro_bais[0]) * DEG2RAD; attitude.gy = (sensor_imu.gyro[1] - gyro_bais[1]) * DEG2RAD; attitude.gz = (sensor_imu.gyro[2] - gyro_bais[2]) * DEG2RAD;
                                attitude.ax = sensor_imu.accl[0]; attitude.ay = sensor_imu.accl[1]; attitude.az = sensor_imu.accl[2];
                            }
                            uorb_check(UORB_ID(sensor_barometer), barometer_sub, &sensor_barometer_update);
                            if (sensor_barometer_update) {
                                uorb_copy(UORB_ID(sensor_barometer), &barometer_sub, &sensor_barometer);
                            }
                            uorb_check(UORB_ID(sensor_magnetometer), magnetometer_sub, &sensor_magnetometer_update);
                            if (sensor_magnetometer_update) {
                                uorb_copy(UORB_ID(sensor_magnetometer), &magnetometer_sub, &sensor_magnetometer);
                                attitude.mx = -sensor_magnetometer.mag[0]; attitude.my = sensor_magnetometer.mag[1]; attitude.mz = sensor_magnetometer.mag[2];
                            }

                            if (sensor_imu_update) {
                                MadgwickAHRSupdate(attitude.gx, attitude.gy, attitude.gz,
                                                   attitude.ax, attitude.ay, attitude.az,
                                                   attitude.mx, attitude.my, attitude.mz);

                                attitude_quaternion.v0            = (int16_t)(roundf(q0 * 10000));
                                attitude_quaternion.v1            = (int16_t)(roundf(q1 * 10000));
                                attitude_quaternion.v2            = (int16_t)(roundf(q2 * 10000));
                                attitude_quaternion.v3            = (int16_t)(roundf(q3 * 10000));
                                attitude_quaternion.fusion_status = 5;
                                data = ano_pack_data(ANO_BROADCAST_ADDR, ATTITUDE_QUATERNION, ANO_ATTITUDE_Q_LENGTH, (uint8_t* )&attitude_quaternion);
                                serial_send(data, data[3] + ANO_ELSE_DATA_PACKET_LENGTH);
                                free(data);

                                q2euler(q0, q1, q2, q3, &attitude.roll, &attitude.pitch, &attitude.yaw);    /* rad */
                                attitude_euler.roll = (int16_t)(roundf(attitude.roll * 57.3 * 100));
                                attitude_euler.pitch = (int16_t)(roundf(attitude.pitch * 57.3 * 100));
                                attitude_euler.yaw = (int16_t)(roundf(attitude.yaw * 57.3 * 100));
                                attitude_euler.fusion_status = 5;
                                data = ano_pack_data(ANO_BROADCAST_ADDR, ATTITUDE_EULER, ANO_ATTITUDE_EULER_LENGTH, (uint8_t* )&attitude_euler);
                                serial_send(data, data[3] + ANO_ELSE_DATA_PACKET_LENGTH);
                                free(data);

                                // printf("roll=%f\r\n", attitude.roll);
                                // printf("pitch=%f\r\n", attitude.pitch);
                                // printf("yaw=%f\r\n", attitude.yaw);
                            }

                            OS_TaskDelay(1);
                            break;
                        }
            default   : {
                            OS_TaskDelay(1);
                            break;
                        }
        }
        g_srs_attitude_counter++;
    }
}

void q2euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw)
{
  *roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f *(q1 * q1 + q2 * q2));

  *pitch = -asin(2.0f * (q0 * q2 - q3 * q1));

  *yaw = -atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
