#include "imu.h"
#include "mpu9250.h"
#include "math.h"

TX_THREAD imu_mag_tcb;
UCHAR imu_mag_stack[IMU_MAG_STACKSIZE];

void imu_mag_entry(ULONG thread_input)
{
  if (mpu9250_init() != 0)
  {
    printf("[sensor]: mpu9250 init error\n");
    return;
  }
  sensor_imu_t sensor_imu;
  sensor_mag_t sensor_mag;
  memset(&sensor_imu, 0, sizeof(sensor_imu_t));
  memset(&sensor_mag, 0, sizeof(sensor_mag_t));

  int count = 0;
  ULONG timestamp, sub_timestamp;
  ULONG delay = 4;
  while (1)
  {
    timestamp = tx_time_get();

    mpu9250_get_gyro((mpu9250_data_t *)&sensor_imu.gyro);
    mpu9250_get_acce((mpu9250_data_t *)&sensor_imu.acce);
    mpu9250_get_mag((mpu9250_data_t *)&sensor_mag.data);

    /* 坐标系转换为前右下 */
    double ax, ay, az;
    ax = sensor_imu.acce[1];
    ay = sensor_imu.acce[0];
    az = sensor_imu.acce[2];
    double pitch, roll;
    pitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.3f;
    roll = atan2(ay, az) * 57.3f;

    if(count++ % 250 == 0)
    {
      printf("pitch:%f, roll:%f", pitch, roll);
      // printf("acce: %.2f %.2f %.2f, gyro: %.4f %.4f %.4f\n", sensor_imu.acce[0], sensor_imu.acce[1], sensor_imu.acce[2],
      //      sensor_imu.gyro[0], sensor_imu.gyro[1], sensor_imu.gyro[2]);
    }

    /* 频率：250Hz */
    /* TODO: 似乎实际OS的tx_thread_sleep函数并不很精准，延时1000，实际在1000上下波动，但总体似乎是小于1000的，可能串口助手的机制不精准 */
    sub_timestamp = tx_time_get() - timestamp;
    if(sub_timestamp < delay)
    {
      tx_thread_sleep(delay - sub_timestamp);
    }
  }
}
