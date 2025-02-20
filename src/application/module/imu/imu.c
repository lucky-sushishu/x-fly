#include "imu.h"

/* Define the thread */
TX_THREAD imu_mag_tcb;
UCHAR imu_mag_stack[IMU_MAG_STACKSIZE];

/* Define the imu queue */
TX_QUEUE queue_comm;
/* Define event flags group */
TX_EVENT_FLAGS_GROUP event_flags_led;
/* Define semaphore */
TX_SEMAPHORE semaphore_imu;
/* Variable */
UCHAR queue_imu_area[3*sizeof(float)*IMU_QUEUE_SIZE];

static float gyro_bais[3];
static mpu9250_imu_data_t mpu9250_imu_data;

/* Function declaration */
void compute_gyro_bais(void);
void q2euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw);

void imu_mag_entry(ULONG thread_input)
{
  /* init */
  UINT status;
  sensor_imu_t sensor_imu = {0};
  sensor_mag_t sensor_mag = {0};
  communication_data_t communication_data = {0};
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  int count = 0;

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
    HAL_I2C_Mem_Read_DMA(&hi2c1, (MPU9250_ADDR<<1), MPU_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&mpu9250_imu_data, 14);
    
    status =  tx_semaphore_get(&semaphore_imu, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
      continue;
    
    sensor_imu.acce[0] = ((float)(((uint16_t)mpu9250_imu_data.accel_xout_h << 8) | mpu9250_imu_data.accel_xout_l)) * MPU9250_ACCE_16G_SEN;
    sensor_imu.acce[1] = ((float)(((uint16_t)mpu9250_imu_data.accel_yout_h << 8) | mpu9250_imu_data.accel_yout_l)) * MPU9250_ACCE_16G_SEN;
    sensor_imu.acce[2] = ((float)(((uint16_t)mpu9250_imu_data.accel_zout_h << 8) | mpu9250_imu_data.accel_zout_l)) * MPU9250_ACCE_16G_SEN;
    sensor_imu.gyro[0] = ((float)(((uint16_t)mpu9250_imu_data.gyro_xout_h << 8) | mpu9250_imu_data.gyro_xout_h)) * MPU9250_GYRO_2000_SEN;
    sensor_imu.gyro[1] = ((float)(((uint16_t)mpu9250_imu_data.gyro_yout_h << 8) | mpu9250_imu_data.gyro_yout_h)) * MPU9250_GYRO_2000_SEN;
    sensor_imu.gyro[2] = ((float)(((uint16_t)mpu9250_imu_data.gyro_zout_h << 8) | mpu9250_imu_data.gyro_zout_h)) * MPU9250_GYRO_2000_SEN;
    
    ax = sensor_imu.acce[0];
    ay = sensor_imu.acce[1];
    az = sensor_imu.acce[2];
    gx = sensor_imu.gyro[0];
    gy = sensor_imu.gyro[1];
    gz = sensor_imu.gyro[2];

    communication_data.imu.acce[0] = -sensor_imu.acce[1];
    communication_data.imu.acce[1] = -sensor_imu.acce[0];
    communication_data.imu.acce[2] = sensor_imu.acce[2];
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

    // if(count++ % 2 == 0)
    // {
    //   printf("ax:%f, ay:%f, az:%f.. gx:%f, gy:%f, gz:%f.. \n", ax, ay, az, gx, gy, gz);
    //   // printf("mx:%f, my:%f, mz:%f\r\n", mx, my, mz);
    // }
    #endif

    #if 0
    mpu9250_get_acce((mpu9250_data_t *)&sensor_imu.acce);
    mpu9250_get_gyro((mpu9250_data_t *)&sensor_imu.gyro);

    /* FRD (right-hand system) */
    // ax = -sensor_imu.acce[1] / G;
    // ay = -sensor_imu.acce[0] / G;
    // az = sensor_imu.acce[2] / G;
    // gx = -sensor_imu.gyro[1] * 57.3;
    // gy = -sensor_imu.gyro[0] * 57.3;
    // gz = sensor_imu.gyro[2] * 57.3;
    ax = sensor_imu.acce[0];
    ay = sensor_imu.acce[1];
    az = sensor_imu.acce[2];
    gx = sensor_imu.gyro[0];
    gy = sensor_imu.gyro[1];
    gz = sensor_imu.gyro[2];

    
    /* 2*10 = 20ms -> 50hz */
    // if(count++ % 2 == 0)
    // {
    //   mpu9250_get_mag((mpu9250_data_t *)&sensor_mag.data);
    //   mx = sensor_mag.data[1] * 0.01; /* uT -> Gs */
    //   my = sensor_mag.data[0] * 0.01;
    //   mz = sensor_mag.data[2] * 0.01;
    //   // printf("mx:%f, my:%f, mz:%f\r\n", mx, my, mz);
    // }

    // MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    // MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, mz, my);

    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    // MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, mz, my);

    communication_data.imu.acce[0] = -sensor_imu.acce[1];
    communication_data.imu.acce[1] = -sensor_imu.acce[0];
    communication_data.imu.acce[2] = sensor_imu.acce[2];
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

    // if(count++ % 200 == 0)
    // {
    //   // printf("ax:%f, ay:%f, az:%f.. gx:%f, gy:%f, gz:%f.. \n", ax, ay, az, gx, gy, gz);
    //   // printf("mx:%f, my:%f, mz:%f\r\n", mx, my, mz);
    // }

    /* TODO : achieve more precise frequency */
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

/* Z-Y-X */
void q2euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw)
{
  *roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f *(q1 * q1 + q2 * q2));

  *pitch = asin(2.0f * (q0 * q2 - q3 * q1));

  *yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
