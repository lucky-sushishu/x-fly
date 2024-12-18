#include "communication.h"

TX_THREAD communication_tcb;
UCHAR communication_stack[COMMUNICATION_STACKSIZE];

void communication_entry(ULONG thread_input)
{
    /* init */
    float r = 1.0f, p = 2.0f, y = 3.0f;
    attitude_euler_t attitude_euler = {0};

    while (1)
    {
        attitude_euler.roll = (int16_t)(roundf(r * 100));
        attitude_euler.pitch = (int16_t)(roundf(p * 100));
        attitude_euler.yaw = (int16_t)(roundf(y * 100));
        attitude_euler.fusion_status = 5;

        uint8_t *data = ano_pack_data(ANO_BROADCAST_ADDR, ATTITUDE_EULER, ANO_ATTITUDE_EULER_LENGTH, (uint8_t* )&attitude_euler);
        HAL_UART_Transmit(&huart1, data, data[3] + ANO_ELSE_DATA_PACKET_LENGTH, 5);

        free(data);
        tx_thread_sleep(10);
    }
}
