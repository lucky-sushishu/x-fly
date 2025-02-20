#include "communication.h"

TX_THREAD communication_tcb;
UCHAR communication_stack[COMMUNICATION_STACKSIZE];

void communication_entry(ULONG thread_input)
{
    /* init */
    UINT status;
    uint8_t *data = NULL;
    communication_data_t communication_data = {0};

    imu_data_t imu_data = {0};
    mag_baro_tmp_data_t mag_baro_tmp_data = {0};
    attitude_euler_t attitude_euler = {0};
    attitude_quaternion_t attitude_quaternion = {0};

    while (1)
    {
        status = tx_queue_receive(&queue_comm, &communication_data, TX_WAIT_FOREVER);
        if(status != TX_SUCCESS)
        {
            continue;
        }

        #if 1
        imu_data.acc_x = (int16_t)(roundf(communication_data.imu.acce[0] * 100));
        imu_data.acc_y = (int16_t)(roundf(communication_data.imu.acce[1] * 100));
        imu_data.acc_z = (int16_t)(roundf(communication_data.imu.acce[2] * 100));
        imu_data.gyr_x = (int16_t)(roundf(communication_data.imu.gyro[0] * 10000));
        imu_data.gyr_y = (int16_t)(roundf(communication_data.imu.gyro[1] * 10000));
        imu_data.gyr_z = (int16_t)(roundf(communication_data.imu.gyro[2] * 10000));
        imu_data.shock_sta = 0;
        data = ano_pack_data(ANO_BROADCAST_ADDR, IMU_DATA, ANO_IMU_DATA_LENGTH, (uint8_t* )&imu_data);
        HAL_UART_Transmit(&huart1, data, data[3] + ANO_ELSE_DATA_PACKET_LENGTH, 5);
        free(data);

        // mag_baro_tmp_data.mag_x = (int16_t)(roundf(communication_data.mag.data[0] * 100));
        // mag_baro_tmp_data.mag_y = (int16_t)(roundf(communication_data.mag.data[1] * 100));
        // mag_baro_tmp_data.mag_z = (int16_t)(roundf(communication_data.mag.data[2] * 100));
        // mag_baro_tmp_data.tmp = 100;
        // data = ano_pack_data(ANO_BROADCAST_ADDR, MAG_BARO_TMP_DATA, ANO_MAG_BARO_TMP_DATA_LENGTH, (uint8_t* )&mag_baro_tmp_data);
        // HAL_UART_Transmit(&huart1, data, data[3] + ANO_ELSE_DATA_PACKET_LENGTH, 5);
        // free(data);
        // // printf("%f %f %f\n", communication_data.mag.data[0], communication_data.mag.data[1], communication_data.mag.data[2]);

        // attitude_euler.roll = (int16_t)(roundf(communication_data.euler_rad.roll * 57.3 * 100));
        // attitude_euler.pitch = (int16_t)(roundf(communication_data.euler_rad.pitch * 57.3 * 100));
        // attitude_euler.yaw = (int16_t)(roundf(communication_data.euler_rad.yaw * 57.3 * 100));
        // attitude_euler.fusion_status = 5;
        // data = ano_pack_data(ANO_BROADCAST_ADDR, ATTITUDE_EULER, ANO_ATTITUDE_EULER_LENGTH, (uint8_t* )&attitude_euler);
        // HAL_UART_Transmit(&huart1, data, data[3] + ANO_ELSE_DATA_PACKET_LENGTH, 5);
        // free(data);

        // attitude_quaternion.v0 = (int16_t)(roundf(communication_data.quaternion.v0 * 10000));
        // attitude_quaternion.v1 = (int16_t)(roundf(communication_data.quaternion.v1 * 10000));
        // attitude_quaternion.v2 = (int16_t)(roundf(communication_data.quaternion.v2 * 10000));
        // attitude_quaternion.v3 = (int16_t)(roundf(communication_data.quaternion.v3 * 10000));
        // attitude_quaternion.fusion_status = 5;
        // data = ano_pack_data(ANO_BROADCAST_ADDR, ATTITUDE_QUATERNION, ANO_ATTITUDE_Q_LENGTH, (uint8_t* )&attitude_quaternion);
        // HAL_UART_Transmit(&huart1, data, data[3] + ANO_ELSE_DATA_PACKET_LENGTH, 5);
        // free(data);

        #endif
    }
}
