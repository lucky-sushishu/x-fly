#include "mpu9250.h"

TX_SEMAPHORE tx_sdriver_input_semaphore;
TX_SEMAPHORE tx_sdriver_output_semaphore;

uint8_t mpu_read_byte(uint8_t addr, uint8_t reg)
{
  uint8_t res = 0;
  i2c_start();
  i2c_send_byte((addr << 1) | 0x00); // 发送器件地址+写命令
  res = i2c_wait_ack();              // 等待应答
                                     //	printf("res is %d\r\n", res);

  i2c_send_byte(reg);   // 写寄存器地址
  res = i2c_wait_ack(); // 等待应答
                        //	printf("res is %d\r\n", res);

  i2c_start();
  i2c_send_byte((addr << 1) | 0x01); // 发送器件地址+读命令
  res = i2c_wait_ack();              // 等待应答
                                     //	printf("res is %d\r\n", res);

  res = i2c_read_byte(0); // 读数据，发送nACK
  i2c_stop();
  return res;
}

uint8_t mpu_write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
  i2c_start();
  i2c_send_byte((addr << 1) | 0x00); // 发送器件地址+写命令
  if (i2c_wait_ack())                // 等待应答
  {
    i2c_stop();
    return 1;
  }
  i2c_send_byte(reg);  // 发送要写入的寄存器地址
  i2c_wait_ack();      // 等待应答
  i2c_send_byte(data); // 发送要写入的数据
  if (i2c_wait_ack())  // 等待应答
  {
    i2c_stop();
    return 1;
  }
  i2c_stop();

  return 0;
}

uint8_t mpu_read_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  i2c_start();
  i2c_send_byte((addr << 1) | 0x00); // 发送器件地址+写命令
  if (i2c_wait_ack())                // 等待应答
  {
    i2c_stop();
    return 1;
  }
  i2c_send_byte(reg); // 发送要读取的起始寄存器地址
  i2c_wait_ack();     // 等待应答
  i2c_start();
  i2c_send_byte((addr << 1) | 0x01); // 发送器件地址+读命令
  i2c_wait_ack();                    // 等待应答

  while (len)
  {
    if (len == 1)
      *buf = i2c_read_byte(0); // 读数据，发送nACK
    else
      *buf = i2c_read_byte(1); // 读数据，发送ACK
    len--;
    buf++;
  }
  i2c_stop();
  return 0;
}

uint8_t mpu_write_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  uint8_t i;
  i2c_start();
  i2c_send_byte((addr << 1) | 0x00); // 发送器件地址+写命令
  if (i2c_wait_ack())                // 等待应答
  {
    i2c_stop();
    return 1;
  }
  i2c_send_byte(reg); // 发送要写入的起始寄存器地址
  i2c_wait_ack();     // 等待应答
  for (i = 0; i < len; i++)
  {
    i2c_send_byte(buf[i]); // 发送要写入的数据
    if (i2c_wait_ack())    // 等待ACK
    {
      i2c_stop();
      return 1;
    }
  }
  i2c_stop();
  return 0;
}

/**
 *range :0,±250deg/s, 1,±500deg/s, 2,±1000deg/s, 3,±2000deg/s
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t mpu_set_gyro_range(uint8_t range)
{
  mpu_write_byte(MPU9250_ADDR, MPU_GYRO_CFG_REG, range << 3);
  return 0;
}

/**
 *range :0,±2g, 1,±4g, 2,±8g, 3,±16g
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t mpu_set_acce_range(uint8_t range)
{
  mpu_write_byte(MPU9250_ADDR, MPU_ACCE_CFG_REG, range << 3);
  return 0;
}

/**
 *des   :设置低通滤波器
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
  uint8_t data = 0;
  if (lpf >= 188)
    data = 1;
  else if (lpf >= 98)
    data = 2;
  else if (lpf >= 42)
    data = 3;
  else if (lpf >= 20)
    data = 4;
  else if (lpf >= 10)
    data = 5;
  else
    data = 6;
  return mpu_write_byte(MPU9250_ADDR, MPU_CFG_REG, data);
}

/**
 *des   :设置采样率
 *rate  :采样率
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t MPU_Set_rate(uint16_t rate)
{
  uint8_t data;
  if (rate > 1000)
    rate = 1000;
  if (rate < 4)
    rate = 1;
  data = 1000 / rate - 1;
  data = mpu_write_byte(MPU9250_ADDR, MPU_SAMPLE_RATE_REG, data);

  return MPU_Set_LPF(rate / 2); // 自动设置LPF为采样率的一半
}

uint8_t mpu9250_init(void)
{
  uint8_t res = 0;
  i2c_init();
  mpu_write_byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0x80); // 复位MPU9250
  delay_ms(100);
  mpu_write_byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU9250 H_RESET会自动清0

  res = mpu_read_byte(MPU9250_ADDR, MPU_WHO_AM_I); // 读取MPU9250的ID
  if (res == MPU9250_ID)
  {
    mpu_write_byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0x03);
    mpu_write_byte(MPU9250_ADDR, MPU_PWR_MGMT2_REG, 0x00); // 开启陀螺仪和加速度计(on)
  }
  else
  {
    return 1;
  }

  mpu_set_gyro_range(3);                                 // 设置陀螺仪的量程为±2000deg/s
  mpu_set_acce_range(3);                                 // 设置加速度计的量程为±16G
  MPU_Set_rate(500);                                     // 设置采样率为500Hz
  mpu_write_byte(MPU9250_ADDR, MPU_INT_EN_REG, 0x00);    // 关闭所有中断
  mpu_write_byte(MPU9250_ADDR, MPU_USER_CTRL_REG, 0x00); // I2C主模式关闭
  mpu_write_byte(MPU9250_ADDR, MPU_FIFO_EN_REG, 0x00);   // 关闭FIFO
  mpu_write_byte(MPU9250_ADDR, MPU_INTBP_CFG_REG, 0x02); // 开启BYPASS模式，可以直接读取磁力计

  res = mpu_read_byte(MAG_ADDR, MAG_WHO_AM_I);
  if (res == AK8963_ID)
  {
    mpu_write_byte(MAG_ADDR, MAG_CNTL1, 0x11); // 设置AK8963为单次测试模式
  }
  else
  {
    return 1;
  }

  return 0;
}

/**
 *return:温度值
 */
int16_t mpu9250_get_temperture(void)
{
  uint8_t buf[2];
  int16_t raw;
  float temp;

  mpu_read_len(MPU9250_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
  raw = ((uint8_t)buf[0] << 8) | buf[1];
  temp = ((double)raw) / 333.87 + 21; // ???(TEMP_degC = ((TEMP_OUT –RoomTemp_Offset)/Temp_Sensitivity) + 21degC)
  return temp * 100;
}

/**
 *des   :得到陀螺仪值(原始值)
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t _mpu9250_get_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[6], res;
  res = mpu_read_len(MPU9250_ADDR, MPU_GYRO_XOUT_H, 6, buf);
  if (res == 0)
  {
    *gx = ((uint16_t)buf[0] << 8) | buf[1];
    *gy = ((uint16_t)buf[2] << 8) | buf[3];
    *gz = ((uint16_t)buf[4] << 8) | buf[5];
  }
  return res;
}

/**
 *des   :得到加速度值(原始值)
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t _mpu9250_get_acce_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t buf[6], res;
  res = mpu_read_len(MPU9250_ADDR, MPU_ACCEL_XOUT_H, 6, buf);
  if (res == 0)
  {
    *ax = ((uint16_t)buf[0] << 8) | buf[1];
    *ay = ((uint16_t)buf[2] << 8) | buf[3];
    *az = ((uint16_t)buf[4] << 8) | buf[5];
  }
  return res;
}

/**
 *des   :得到磁力计值(原始值)
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t _mpu9250_get_mag_raw(int16_t *mx, int16_t *my, int16_t *mz)
{
  uint8_t buf[6], res;
  res = mpu_read_len(MAG_ADDR, MAG_HXL, 6, buf);
  if (res == 0)
  {
    *mx = ((uint16_t)buf[1] << 8) | buf[0];
    *my = ((uint16_t)buf[3] << 8) | buf[2];
    *mz = ((uint16_t)buf[5] << 8) | buf[4];
  }
  mpu_write_byte(MAG_ADDR, MAG_CNTL1, 0x11); // AK8963读完之后都需要重新设置为单次测试模式
  return res;
}

/**
 *des   :得到陀螺仪值(单位：弧度/秒)
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t mpu9250_get_gyro(mpu9250_data_t *buf)
{
  mpu9250_raw_data_t tmp;

  if (_mpu9250_get_gyro_raw(&tmp.x, &tmp.y, &tmp.z) == 0)
  {
    buf->x = ((float)tmp.x) * MPU9250_GYRO_2000_SEN;
    buf->y = ((float)tmp.y) * MPU9250_GYRO_2000_SEN;
    buf->z = ((float)tmp.z) * MPU9250_GYRO_2000_SEN;
    return 0;
  }
  return 1;
}

/**
 *des   :得到加速度值(单位：米/平方秒)
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t mpu9250_get_acce(mpu9250_data_t *buf)
{
  mpu9250_raw_data_t tmp;

  if (_mpu9250_get_acce_raw(&tmp.x, &tmp.y, &tmp.z) == 0)
  {
    buf->x = ((float)tmp.x) * MPU9250_ACCE_16G_SEN;
    buf->y = ((float)tmp.y) * MPU9250_ACCE_16G_SEN;
    buf->z = ((float)tmp.z) * MPU9250_ACCE_16G_SEN;
    return 0;
  }
  return 1;
}

/**
 *des   :得到磁力计值(单位：uT)
 *return:返回0设置成功，返回其他设置失败
 */
uint8_t mpu9250_get_mag(mpu9250_data_t *buf)
{
  mpu9250_raw_data_t tmp;

  if (_mpu9250_get_mag_raw(&tmp.x, &tmp.y, &tmp.z) == 0)
  {
    buf->x = ((float)tmp.x) * 0.15;
    buf->y = ((float)tmp.y) * 0.15;
    buf->z = ((float)tmp.z) * 0.15;
    return 0;
  }
  return 1;
}

VOID tx_sdriver_initialize(VOID)
{
  /* Initialize the two counting semaphores used to control
  the simple driver I/O. */
  tx_semaphore_create(&tx_sdriver_input_semaphore,
                      "simple driver input semaphore", 0);
  tx_semaphore_create(&tx_sdriver_output_semaphore,
                      "simple driver output semaphore", 1);

  /* Setup interrupt vectors for input and output ISRs.
  The initial vector handling should call the ISRs
  defined in this file. */

  /* Configure serial device hardware for RX/TX interrupt
  generation, baud rate, stop bits, etc. */
}
