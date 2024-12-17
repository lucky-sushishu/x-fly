#include "imu.h"
#include "mpu9250.h"
#include "math.h"
#include "MahonyAHRS.h"

TX_THREAD imu_mag_tcb;
UCHAR imu_mag_stack[IMU_MAG_STACKSIZE];

static sensor_imu_t sensor_imu;
static float acce_bais[3], gyro_bais[3];

static float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

void compute_imu_bais(void);
void q2euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw);

void imu_mag_entry(ULONG thread_input)
{
  /* init */
  float ax, ay, az, gx, gy, gz;

  int count = 0;

  if (mpu9250_init() != 0)
  {
    /* TODO : give led a message display imu init error */
    printf("[sensor]: mpu9250 init error\n");
    return;
  }

  compute_imu_bais();

  while (1)
  {
    mpu9250_get_acce((mpu9250_data_t *)&sensor_imu.acce);
    mpu9250_get_gyro((mpu9250_data_t *)&sensor_imu.gyro);

    /* right-hand system */
    ax = -(sensor_imu.acce[0] - acce_bais[0]) / 9.81f;
    ay = -(sensor_imu.acce[1] - acce_bais[1]) / 9.81f;
    az = (sensor_imu.acce[2] - acce_bais[2]) / 9.81f;
    gx = -(sensor_imu.gyro[0] - gyro_bais[0]);
    gy = -(sensor_imu.gyro[1] - gyro_bais[1]);
    gz = (sensor_imu.gyro[2] - gyro_bais[2]);

    MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);

    if(count++ % 250 == 0)
    {
      printf("ax:%f, ay:%f, az:%f.. gx:%f, gy:%f, gz:%f\r\n", ax, ay, az, gx, gy, gz);
      q2euler(q0, q1, q2, q3, &roll, &pitch, &yaw);
      printf("roll:%f, pitch:%f yaw:%f\r\n", roll * 57.3, pitch * 57.3, yaw * 57.3);
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
