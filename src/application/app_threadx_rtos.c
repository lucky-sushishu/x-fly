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

typedef struct sensor_imu_s
{
  float gyro[3];
  float acce[3];
} sensor_imu_t;

typedef struct sensor_mag_s
{
  float x;
  float y;
  float z;
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
  if (mpu9250_init() != 0)
  {
    printf("[sensor]: mpu9250 init error\n");
  }
  mpu9250_data_t mpu9250_data;
  sensor_imu_t sensor_imu;
  sensor_mag_t sensor_mag;
  while (1)
  {
    mpu9250_get_gyro(&mpu9250_data);
    sensor_imu.gyro[0] = mpu9250_data.x;
    sensor_imu.gyro[1] = mpu9250_data.y;
    sensor_imu.gyro[2] = mpu9250_data.z;
    mpu9250_get_acce(&mpu9250_data);
    sensor_imu.acce[0] = mpu9250_data.x;
    sensor_imu.acce[1] = mpu9250_data.y;
    sensor_imu.acce[2] = mpu9250_data.z;
    mpu9250_get_mag(&mpu9250_data);
    sensor_mag.x = mpu9250_data.x;
    sensor_mag.y = mpu9250_data.y;
    sensor_mag.z = mpu9250_data.z;
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
