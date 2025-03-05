#ifndef _THREAD_H
#define _THREAD_H

#include "tx_api.h"
#include "sensor.h"
#include "communication.h"
#include "led.h"
#include "cli.h"

#define COMMUNICATION_QUEUE_SIZE 100

/* Define rtos event flags */
#define IMU_INIT_SUCCEED (0x1 << 0)
#define IMU_INIT_ERROR   (0x1 << 1)

extern TX_QUEUE queue_comm;
extern UCHAR queue_communication_area[3*sizeof(float)*COMMUNICATION_QUEUE_SIZE];
extern TX_EVENT_FLAGS_GROUP event_flags_led;
extern TX_SEMAPHORE semaphore_imu;

#endif
