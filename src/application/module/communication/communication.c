#include "communication.h"

TX_THREAD communication_tcb;
UCHAR communication_stack[COMMUNICATION_STACKSIZE];

void communication_entry(ULONG thread_input)
{
    /* init */
    UINT status;
    #if USE_EULER_RAD
    euler_rad_t euler_rad = {0};
    attitude_euler_t attitude_euler = {0};
    #else
    quaternion_t quaternion = {0};
    attitude_q_t attitude_q = {0};
    #endif

    while (1)
    {
        #if USE_EULER_RAD
        status = tx_queue_receive(&queue_imu, &euler_rad, TX_WAIT_FOREVER);
        if(status != TX_SUCCESS)
        {
            continue;
            // printf("roll:%f, pitch:%f yaw:%f\r\n", euler_rad.roll, euler_rad.pitch, euler_rad.yaw);
        }
        attitude_euler.roll = (int16_t)(roundf(euler_rad.roll * 57.3 * 100));
        attitude_euler.pitch = (int16_t)(roundf(euler_rad.pitch * 57.3 * 100));
        attitude_euler.yaw = (int16_t)(roundf(euler_rad.yaw * 57.3 * 100));
        attitude_euler.fusion_status = 5;
        uint8_t *data = ano_pack_data(ANO_BROADCAST_ADDR, ATTITUDE_EULER, ANO_ATTITUDE_EULER_LENGTH, (uint8_t* )&attitude_euler);
        #else

        status = tx_queue_receive(&queue_imu, &quaternion, TX_WAIT_FOREVER);
        if(status != TX_SUCCESS)
        {
            continue;
            // printf("%f %f %f %f\r\n", quaternion.v0, quaternion.v1, quaternion.v2, quaternion.v3);
        }
        attitude_q.v0 = (int16_t)(roundf(quaternion.v0 * 10000));
        attitude_q.v1 = (int16_t)(roundf(quaternion.v1 * 10000));
        attitude_q.v2 = (int16_t)(roundf(quaternion.v2 * 10000));
        attitude_q.v3 = (int16_t)(roundf(quaternion.v3 * 10000));
        attitude_q.fusion_status = 5;
        uint8_t *data = ano_pack_data(ANO_BROADCAST_ADDR, ATTITUDE_Q, ANO_ATTITUDE_Q_LENGTH, (uint8_t* )&attitude_q);
        #endif

        HAL_UART_Transmit(&huart1, data, data[3] + ANO_ELSE_DATA_PACKET_LENGTH, 5);

        free(data);
    }
}
