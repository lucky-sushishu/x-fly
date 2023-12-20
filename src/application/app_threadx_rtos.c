/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
   threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
   byte pool, and block pool.  */

#include "tx_api.h"
#include "gpio.h"
#include "mpu9250.h"

#define APP1_PRIO 15
#define APP1_STACKSIZE 1024
static TX_THREAD app1_tcb;
static UCHAR app1_stack[APP1_STACKSIZE];

#define MPU9250_PRIO 14
#define MPU9250_STACKSIZE 1024
static TX_THREAD mpu9250_tcb;
static UCHAR  mpu9250_stack[MPU9250_STACKSIZE];

void app1_led(ULONG thread_input)
{
	
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_Delay(500);
	}
}

void mpu9250(ULONG thread_input)
{
	i2c_init();
	while(1)
	{
		if(mpu9250_work_mode_init())
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		HAL_Delay(100);
	}
}

void tx_application_define(void *first_unused_memory)
{

	UINT status = tx_thread_create(&app1_tcb, "app1_led", app1_led, 0,
										&app1_stack[0], APP1_STACKSIZE,
										APP1_PRIO, APP1_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);
	tx_thread_create(&mpu9250_tcb, "mpu9250", mpu9250, 0,
										&mpu9250_stack[0], MPU9250_STACKSIZE,
										MPU9250_PRIO, MPU9250_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);
}

