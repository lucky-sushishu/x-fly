#include "imu.h"

/* Define the thread */
TX_THREAD imu_mag_tcb;
UCHAR imu_mag_stack[IMU_MAG_STACKSIZE];

/* Define the imu queue */
TX_QUEUE queue_imu;
UCHAR queue_imu_area[3*sizeof(float)*IMU_QUEUE_SIZE];

/* Define event flags group */
TX_EVENT_FLAGS_GROUP event_flags_led;

static sensor_imu_t sensor_imu;
static sensor_mag_t sensor_mag;
static float acce_bais[3], gyro_bais[3];

/* Function declaration */
void compute_imu_bais(void);
void q2euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw);

void imu_mag_entry(ULONG thread_input)
{
  /* init */
  UINT status;
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  euler_rad_t euler_rad = {0};

  int count = 0;

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

  compute_imu_bais();

  while (1)
  {
    mpu9250_get_acce((mpu9250_data_t *)&sensor_imu.acce);
    mpu9250_get_gyro((mpu9250_data_t *)&sensor_imu.gyro);

    /* FRD (right-hand system) */
    ax = -(sensor_imu.acce[1] - acce_bais[1]) / G;
    ay = -(sensor_imu.acce[0] - acce_bais[0]) / G;
    az = (sensor_imu.acce[2] - acce_bais[2]) / G;
    gx = -(sensor_imu.gyro[1] - gyro_bais[1]);
    gy = -(sensor_imu.gyro[0] - gyro_bais[0]);
    gz = -(sensor_imu.gyro[2] - gyro_bais[2]);
    
    /* 2*10 = 20ms -> 50hz */
    if(count++ % 10 == 0)
    {
      mpu9250_get_mag((mpu9250_data_t *)&sensor_mag.data);
      mx = -sensor_mag.data[0] * 0.01; /* uT -> Gs */
      my = -sensor_mag.data[1] * 0.01;
      mz = sensor_mag.data[2] * 0.01;
    }

    MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, mz, my);

    q2euler(q0, q1, q2, q3, &euler_rad.roll, &euler_rad.pitch, &euler_rad.yaw);
    
    status =  tx_queue_send(&queue_imu, &euler_rad, TX_WAIT_FOREVER);

    if(count++ % 250 == 0)
    {
      // printf("ax:%f, ay:%f, az:%f.. gx:%f, gy:%f, gz:%f.. ", ax, ay, az, gx, gy, gz);
      // printf("mx:%f, my:%f, mz:%f\r\n", mx, my, mz);
      // printf("roll:%f, pitch:%f yaw:%f\r\n", euler_rad.roll, euler_rad.pitch, euler_rad.yaw);
    }

    /* TODO : achieve more precise frequency */

    tx_thread_sleep(2);
  }
}

void compute_imu_bais(void)
{
  int samples = 0;
  ULONG timestamp = tx_time_get();
  while((tx_time_get() - timestamp) < 1000)
  {
    mpu9250_get_acce((mpu9250_data_t *)&sensor_imu.acce);
    mpu9250_get_gyro((mpu9250_data_t *)&sensor_imu.gyro);

    acce_bais[0] = sensor_imu.acce[0];
    acce_bais[1] = sensor_imu.acce[1];
    acce_bais[2] = sensor_imu.acce[2];
    gyro_bais[0] = sensor_imu.gyro[0];
    gyro_bais[1] = sensor_imu.gyro[1];
    gyro_bais[2] = sensor_imu.gyro[2];
    
    samples++;
    tx_thread_sleep(2);
  }

  if(samples != 0)
  {
    acce_bais[0] = acce_bais[0] / samples;
    acce_bais[1] = acce_bais[1] / samples;
    acce_bais[2] = acce_bais[2] / samples;
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
