#include "sensor.h"

/* Define the thread */
TX_THREAD sensor_tcb;
UCHAR sensor_stack[SENSOR_STACKSIZE];

static float gyro_bais[3];
static mpu9250_imu_data_t mpu9250_imu_data;

extern I2C_HandleTypeDef hi2c1;

/* Function declaration */
void compute_gyro_bais(void);
void q2euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw);

void sensor_entry(ULONG thread_input)
{
  /* init */
  UINT status;
  sensor_imu_t sensor_imu = {0};
  sensor_mag_t sensor_mag = {0};
  communication_data_t communication_data = {0};
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mx = 0, my = 0, mz = 0;
  int count = 0, rate = 0;

  uint8_t mag_data_tmp[6] = {0}, mag_addr_id = 0;

  #if 1
  if (mpu9250_init() != 0)
  {
    /* Set event flag imu to wakeup thread led.  */
    status =  tx_event_flags_set(&event_flags_led, IMU_INIT_ERROR, TX_OR);
    return;
  }
  else
  {
    /* Set event flag imu to wakeup thread led.  */
    status =  tx_event_flags_set(&event_flags_led, IMU_INIT_SUCCEED, TX_OR);
  }

  compute_gyro_bais();
  #endif

  while (1)
  {
    #if 1
    HAL_I2C_Mem_Read_DMA(&hi2c1, (MPU9250_ADDR<<1), MPU_ACCL_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&mpu9250_imu_data, 14);
    
    status =  tx_semaphore_get(&semaphore_imu, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
      continue;
    
    sensor_imu.accl[0] = ((float)(((int16_t)mpu9250_imu_data.accl_xout_h << 8) | mpu9250_imu_data.accl_xout_l)) * MPU9250_ACCL_16G_SEN;
    sensor_imu.accl[1] = ((float)(((int16_t)mpu9250_imu_data.accl_yout_h << 8) | mpu9250_imu_data.accl_yout_l)) * MPU9250_ACCL_16G_SEN;
    sensor_imu.accl[2] = ((float)(((int16_t)mpu9250_imu_data.accl_zout_h << 8) | mpu9250_imu_data.accl_zout_l)) * MPU9250_ACCL_16G_SEN;
    sensor_imu.gyro[0] = ((float)(((int16_t)mpu9250_imu_data.gyro_xout_h << 8) | mpu9250_imu_data.gyro_xout_l)) * MPU9250_GYRO_2000_SEN - gyro_bais[0];
    sensor_imu.gyro[1] = ((float)(((int16_t)mpu9250_imu_data.gyro_yout_h << 8) | mpu9250_imu_data.gyro_yout_l)) * MPU9250_GYRO_2000_SEN - gyro_bais[1];
    sensor_imu.gyro[2] = ((float)(((int16_t)mpu9250_imu_data.gyro_zout_h << 8) | mpu9250_imu_data.gyro_zout_l)) * MPU9250_GYRO_2000_SEN - gyro_bais[2];

    /* Left system, Front-Left-Up */
    ax = -sensor_imu.accl[1];
    ay = sensor_imu.accl[0];
    az = sensor_imu.accl[2];
    gx = -sensor_imu.gyro[1];
    gy = sensor_imu.gyro[0];
    gz = sensor_imu.gyro[2];

    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    // MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);

    communication_data.imu.accl[0] = ax;
    communication_data.imu.accl[1] = ay;
    communication_data.imu.accl[2] = az;
    communication_data.imu.gyro[0] = gx;
    communication_data.imu.gyro[1] = gy;
    communication_data.imu.gyro[2] = gz;
    communication_data.mag.data[0] = mx;
    communication_data.mag.data[1] = my;
    communication_data.mag.data[2] = mz;
    q2euler(q0, q1, q2, q3, &communication_data.euler_rad.roll, &communication_data.euler_rad.pitch, &communication_data.euler_rad.yaw);
    communication_data.quaternion.v0 = q0;
    communication_data.quaternion.v1 = q1;
    communication_data.quaternion.v2 = q2;
    communication_data.quaternion.v3 = q3;
    status = tx_queue_send(&queue_comm, &communication_data, TX_WAIT_FOREVER);

    // if(count++ % 500 == 0)
    // {
    //   printf("ax:%f, ay:%f, az:%f.. gx:%f, gy:%f, gz:%f.. mx:%f, my:%f, mz:%f.. \n", ax, ay, az, gx, gy, gz, mx, my, mz);
    //   // printf("mx:%f, my:%f, mz:%f.. \n", mx, my, mz);
    // }

    // if(rate++ % 50 == 0)
    // {
    //   tx_thread_sleep(1);
    //   mpu9250_get_mag((mpu9250_data_t *)&sensor_mag.data);
    //   mx = -sensor_mag.data[0]; /* uT -> Gs */
    //   my = -sensor_mag.data[2];
    //   mz = -sensor_mag.data[1];
    //   tx_thread_sleep(1);
    // }
    // else
    // {
    //   tx_thread_sleep(2);
    // }
    #endif
    tx_thread_sleep(2);
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
      tx_semaphore_put(&semaphore_imu);
    }
}

void compute_gyro_bais(void)
{
  int samples = 0;
  float gyro_temp[3] = {0};
  ULONG timestamp = tx_time_get();

  while((tx_time_get() - timestamp) < 1000)
  {
    mpu9250_get_gyro((mpu9250_data_t *)&gyro_temp);

    gyro_bais[0] += gyro_temp[0];
    gyro_bais[1] += gyro_temp[1];
    gyro_bais[2] += gyro_temp[2];
    
    samples++;
    tx_thread_sleep(2);
  }

  if(samples != 0)
  {
    gyro_bais[0] = gyro_bais[0] / samples;
    gyro_bais[1] = gyro_bais[1] / samples;
    gyro_bais[2] = gyro_bais[2] / samples;
  }
}

void q2euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw)
{
  *roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f *(q1 * q1 + q2 * q2));

  *pitch = -asin(2.0f * (q0 * q2 - q3 * q1));

  *yaw = -atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
