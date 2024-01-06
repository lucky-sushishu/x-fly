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
static UCHAR mpu9250_stack[MPU9250_STACKSIZE];


typedef struct sensor_imu_s {
	short gyro[3];
	short acce[3];
	short temp;
} sensor_imu_t;

typedef struct sensor_mag_s {
	short mag[3];
} sensor_mag_t;


void app1_led(ULONG thread_input)
{

	while (1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		tx_thread_sleep(100);
	}
}

void mpu9250(ULONG thread_input)
{
	if (mpu9250_init() != 0) {
		printf("[sensor]: mpu9250 init error\n");
	}
  sensor_imu_t sensor_imu;
  sensor_mag_t sensor_mag;
	while (1)
	{
    MPU_Get_Gyro(&sensor_imu.gyro[0], &sensor_imu.gyro[1], &sensor_imu.gyro[2]);
    MPU_Get_Acce(&sensor_imu.acce[0], &sensor_imu.acce[1], &sensor_imu.acce[2]);
    MPU_Get_Mag(&sensor_mag.mag[0], &sensor_mag.mag[1], &sensor_mag.mag[2]);
    tx_thread_sleep(1);
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
