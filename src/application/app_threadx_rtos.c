/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
   threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
   byte pool, and block pool.  */

#include "tx_api.h"
#include "gpio.h"

#define APP1_PRIO 15
#define APP1_STACKSIZE 1024
static TX_THREAD app1_tcb;
static UCHAR app1_stack[APP1_STACKSIZE];

void app1_led(ULONG thread_input)
{
	
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_Delay(500);
	}
}

void tx_application_define(void *first_unused_memory)
{

	UINT status = tx_thread_create(&app1_tcb, "app1_led", app1_led, 0,
										&app1_stack[0], APP1_STACKSIZE,
										APP1_PRIO, APP1_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);
}

