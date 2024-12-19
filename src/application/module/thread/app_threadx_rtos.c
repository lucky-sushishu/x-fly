/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
   threads of different priorities, using a message queue, semaphore, mutex, event flags group,
   byte pool, and block pool.  */

#include "tx_api.h"
#include "gpio.h"
#include "imu.h"
#include "communication.h"

#define LED_PRIO 15
#define LED_STACKSIZE 1024
static TX_THREAD led_tcb;
static UCHAR led_stack[LED_STACKSIZE];

void led_entry(ULONG thread_input)
{

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    tx_thread_sleep(100);
  }
}

void tx_application_define(void *first_unused_memory)
{
  tx_thread_create(&led_tcb, "led", led_entry, 0,
                                &led_stack[0], LED_STACKSIZE,
                                LED_PRIO, LED_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);
  tx_thread_create(&imu_mag_tcb, "imu_mag", imu_mag_entry, 0,
                                &imu_mag_stack[0], IMU_MAG_STACKSIZE,
                                IMU_MAG_PRIO, IMU_MAG_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);
  tx_thread_create(&communication_tcb, "communication", communication_entry, 0,
                                &communication_stack[0], COMMUNICATION_STACKSIZE,
                                COMMUNICATION_PRIO, COMMUNICATION_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);
  /* Create the message imu queue shared by imu_mag and communication.  */
    tx_queue_create(&queue_imu, "queue imu", 3*TX_1_ULONG, queue_imu_area, 3*sizeof(float)*IMU_QUEUE_SIZE);
}
