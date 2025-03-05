#include "bsp_software_i2c.h"

static void i2c_io_config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PB8, PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
}

void i2c_init(void)
{
	i2c_io_config();
}

/* pull up sda and scl, pull down sda, then pull down scl */
void i2c_start(void)
{
	SDA_OUT();
	I2C_SCL_H;
	I2C_SDA_H;
	delay_us(4);

	I2C_SDA_L;
	delay_us(4);

	I2C_SCL_L;
}

void i2c_stop(void)
{
	SDA_OUT();
	I2C_SDA_L;
	I2C_SCL_L;
	delay_us(4);

	//	I2C_SDA_L;
	I2C_SCL_H;
	delay_us(4);
	I2C_SDA_H;
}

/* return 0 is succeed, 1 is failed */
uint8_t i2c_wait_ack(void)
{
	uint8_t ucErrTime;
	SDA_IN();

	I2C_SDA_H;
	delay_us(1);
	I2C_SCL_H;
	delay_us(1);

	while (I2C_SDA_GET)
	{
		ucErrTime++;
		if (ucErrTime > 250)
		{
			i2c_stop();
			return 1;
		}
	}
	I2C_SCL_L;
	return 0;
}

/* pull down sda, scl pull from high to low */
void i2c_ack(void)
{
	I2C_SCL_L;

	SDA_OUT();
	I2C_SDA_L;
	delay_us(2);

	I2C_SCL_H;
	delay_us(2);

	I2C_SCL_L;
	delay_us(2);
}

void i2c_nack(void)
{
	I2C_SCL_L;
	SDA_OUT();
	I2C_SDA_H;
	delay_us(2);

	I2C_SCL_H;
	delay_us(2);
	I2C_SCL_L;
}

void i2c_send(uint8_t SendByte)
{
	uint8_t i;
	SDA_OUT();

	I2C_SDA_L;
	for (i = 0; i < 8; i++)
	{
		if ((SendByte & 0x80) >> 7)
		{
			I2C_SDA_H;
		}
		else
		{
			I2C_SDA_L;
		}
		delay_us(2);
		I2C_SCL_H;
		delay_us(2);
		I2C_SCL_L;
		delay_us(2);

		SendByte <<= 1;
	}
}

uint8_t i2c_read(uint8_t ack)
{
	uint8_t i, vaule = 0;
	SDA_IN();

	for (i = 0; i < 8; i++)
	{
		I2C_SCL_L;
		delay_us(2);
		I2C_SCL_H;

		vaule <<= 1;
		if (I2C_SDA_GET)
			vaule++;
		delay_us(1);
	}
	if (!ack)
		i2c_nack();
	else
		i2c_ack();

	return vaule;
}

uint8_t i2c_read_byte(uint8_t addr, uint8_t reg)
{
  uint8_t res = 0;
  i2c_start();
  i2c_send((addr << 1) | 0x00); // 发送器件地址+写命令
  res = i2c_wait_ack();              // 等待应答
                                     //	printf("res is %d\r\n", res);

  i2c_send(reg);   // 写寄存器地址
  res = i2c_wait_ack(); // 等待应答
                        //	printf("res is %d\r\n", res);

  i2c_start();
  i2c_send((addr << 1) | 0x01); // 发送器件地址+读命令
  res = i2c_wait_ack();              // 等待应答
                                     //	printf("res is %d\r\n", res);

  res = i2c_read(0); // 读数据，发送nACK
  i2c_stop();
  return res;
}

uint8_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
  i2c_start();
  i2c_send((addr << 1) | 0x00); // 发送器件地址+写命令
  if (i2c_wait_ack())                // 等待应答
  {
    i2c_stop();
    return 1;
  }
  i2c_send(reg);  // 发送要写入的寄存器地址
  i2c_wait_ack();      // 等待应答
  i2c_send(data); // 发送要写入的数据
  if (i2c_wait_ack())  // 等待应答
  {
    i2c_stop();
    return 1;
  }
  i2c_stop();

  return 0;
}

uint8_t i2c_read_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  i2c_start();
  i2c_send((addr << 1) | 0x00); // 发送器件地址+写命令
  if (i2c_wait_ack())                // 等待应答
  {
    i2c_stop();
    return 1;
  }
  i2c_send(reg); // 发送要读取的起始寄存器地址
  i2c_wait_ack();     // 等待应答
  i2c_start();
  i2c_send((addr << 1) | 0x01); // 发送器件地址+读命令
  i2c_wait_ack();                    // 等待应答

  while (len)
  {
    if (len == 1)
      *buf = i2c_read(0); // 读数据，发送nACK
    else
      *buf = i2c_read(1); // 读数据，发送ACK
    len--;
    buf++;
  }
  i2c_stop();
  return 0;
}

uint8_t i2c_write_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  uint8_t i;
  i2c_start();
  i2c_send((addr << 1) | 0x00); // 发送器件地址+写命令
  if (i2c_wait_ack())                // 等待应答
  {
    i2c_stop();
    return 1;
  }
  i2c_send(reg); // 发送要写入的起始寄存器地址
  i2c_wait_ack();     // 等待应答
  for (i = 0; i < len; i++)
  {
    i2c_send(buf[i]); // 发送要写入的数据
    if (i2c_wait_ack())    // 等待ACK
    {
      i2c_stop();
      return 1;
    }
  }
  i2c_stop();
  return 0;
}
