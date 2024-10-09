#include "imu.h"
#include "mpu9250.h"
#include "math.h"

TX_THREAD imu_mag_tcb;
UCHAR imu_mag_stack[IMU_MAG_STACKSIZE];

static sensor_imu_t sensor_imu;
static float acce_bais[3], gyro_bais[3];
static float acce_roll = 0, acce_pitch = 0, gyro_roll = 0, gyro_pitch = 0;

static void compute_imu_bais(void)
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

void imu_mag_entry(ULONG thread_input)
{
  if (mpu9250_init() != 0)
  {
    printf("[sensor]: mpu9250 init error\n");
    return;
  }

  float ax, ay, az, gx, gy, gz;

  int count = 0;
  ULONG timestamp, sub_timestamp;
  ULONG delay = 2;

  compute_imu_bais();
  printf("bais: acce: %f %f %f, gyro: %f %f %f\r\n", 
          acce_bais[0], acce_bais[1], acce_bais[2], gyro_bais[0], gyro_bais[1], gyro_bais[2]);

  while (1)
  {
    // timestamp = tx_time_get();

    mpu9250_get_acce((mpu9250_data_t *)&sensor_imu.acce);
    mpu9250_get_gyro((mpu9250_data_t *)&sensor_imu.gyro);

    /* 坐标系转换为前右下 */
    ax = -(sensor_imu.acce[1] - acce_bais[1]);
    ay = -(sensor_imu.acce[0] - acce_bais[0]);
    az = -(sensor_imu.acce[2] - acce_bais[2]);
    gx = -(sensor_imu.gyro[1] - gyro_bais[1]);
    gy = -(sensor_imu.gyro[0] - gyro_bais[0]);
    gz = -(sensor_imu.gyro[2] - gyro_bais[2]);

    acce_roll = atan2(-ay, -az) * 57.3f;
    acce_pitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.3f;

    gyro_roll += gx * 57.3f * 0.002;
    gyro_pitch += gy * 57.3f * 0.002;

    if(count++ % 250 == 0)
    {
      printf("acce: roll:%f, pitch:%f, gyro: roll:%f, pitch:%f\r\n", acce_roll, acce_pitch, gyro_roll, gyro_pitch);
      // printf("acce: %.2f %.2f %.2f, gyro: %.4f %.4f %.4f\n", sensor_imu.acce[0], sensor_imu.acce[1], sensor_imu.acce[2],
      //      sensor_imu.gyro[0], sensor_imu.gyro[1], sensor_imu.gyro[2]);
    }

    /* 频率：250Hz */
    /* TODO: 似乎实际OS的tx_thread_sleep函数并不很精准，延时1000，实际在1000上下波动，但总体似乎是小于1000的，可能串口助手的机制不精准 */
    // sub_timestamp = tx_time_get() - timestamp;
    // if(sub_timestamp < delay)
    // {
    //   tx_thread_sleep(delay - sub_timestamp);
    // }

    tx_thread_sleep(2);
  }
}
