#include "thread.h"

/* Message queue: thread imu and communication */
TX_QUEUE queue_comm;
UCHAR queue_communication_area[3*sizeof(float)*COMMUNICATION_QUEUE_SIZE];
/* Define event flags group */
TX_EVENT_FLAGS_GROUP event_flags_led;
/* Semaphore: sync i2c+dma and thread */
TX_SEMAPHORE semaphore_imu;

void tx_application_define(void *first_unused_memory)
{
  tx_thread_create(&led_tcb, "led", led_entry, 0,
                                &led_stack[0], LED_STACKSIZE,
                                LED_PRIO, LED_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

  tx_thread_create(&sensor_tcb, "sensor", sensor_entry, 0,
                                &sensor_stack[0], SENSOR_STACKSIZE,
                                SENSOR_PRIO, SENSOR_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

  tx_thread_create(&communication_tcb, "communication", communication_entry, 0,
                                &communication_stack[0], COMMUNICATION_STACKSIZE,
                                COMMUNICATION_PRIO, COMMUNICATION_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

  tx_thread_create(&cli_tcb, "cli", cli_entry, 0,
                                &cli_stack[0], CLI_STACKSIZE,
                                CLI_PRIO, CLI_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Create the comm message queue shared by sensor and communication.  */
  tx_queue_create(&queue_comm, "comm queue", (sizeof(communication_data_t) / 4)*TX_1_ULONG, queue_communication_area, (sizeof(communication_data_t) / 4)*sizeof(float)*COMMUNICATION_QUEUE_SIZE);

  /* Create the event flags group used by threads sensor and led.  */
  tx_event_flags_create(&event_flags_led, "led event flags ");

  /* Create the semaphore used by sensor and i2c dma rx transfer completed callback. */
  tx_semaphore_create(&semaphore_imu, "imu semaphore", 1);
}
