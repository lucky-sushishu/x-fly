#include "i2c.h"

void delay_ms(uint16_t ms)
{
//	uint16_t i, j;
//	for(j = ms; j > 0; j--)
//		for(i = 0; i < 1000; i++);
	HAL_Delay(ms);
}

static void i2c_delay_us(uint32_t us)
{
	uint32_t i;
	/* 100Mhz: 1 nop, 1 us */
	for(i = 0; i < us; i++)
	{
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
		__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();__NOP(); __NOP();
	}
}

static void i2c_delay(void)
{
	i2c_delay_us(3);
}

static void i2c_io_config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB8, PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* pull up sda and scl, pull down sda, then pull down scl */
static void i2c_start(void)
{
	I2C_SCL_H;
	I2C_SDA_H;
	i2c_delay();
	
	I2C_SDA_L;
	i2c_delay();
	
	I2C_SCL_L;
	i2c_delay();
}

/* pull down sda, scl pull from high to low */
static void i2c_ack(void)
{
	I2C_SDA_L;
	i2c_delay();
	
	I2C_SCL_H;
	i2c_delay();
	
	I2C_SCL_L;
	i2c_delay();
	
	I2C_SDA_H;
}

static void i2c_nack(void)
{
	I2C_SDA_H;
	i2c_delay();
	
	I2C_SCL_H;
	i2c_delay();
	
	I2C_SCL_L;
	i2c_delay();
}

static uint8_t i2c_wait_ack(void)
{
	uint8_t result;
	
	I2C_SDA_H;
	i2c_delay();
	I2C_SCL_H;
	i2c_delay();
	
	SDA_IN();
	if(I2C_SDA_GET) {
		SDA_OUT();
		return 1;
	}
	else {
		result = 0;
	}
	
	I2C_SCL_L;
	i2c_delay();
	
	SDA_OUT();
	return result;
}

static void i2c_stop(void)
{
	I2C_SDA_L;
	I2C_SCL_H;
	i2c_delay();
	
	I2C_SDA_H;
}


void i2c_init(void)
{
	i2c_io_config();
}


void i2c_send_byte(uint8_t SendByte)
{
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		if(SendByte & 0x80) {
			I2C_SDA_H;
		}
		else {
			I2C_SDA_L;
		}
		i2c_delay();
		I2C_SCL_H;
		i2c_delay();
		I2C_SCL_L;
		
		if(i == 7)
		{
			I2C_SDA_H; /* release bus */
		}
		SendByte <<= 1;
		i2c_delay();
	}
}

uint8_t i2c_receive_byte(void)
{
	uint8_t i;
	uint8_t vaule = 0;
	SDA_IN();
	
	for(i = 0; i < 8; i++)
	{
		vaule <<= 1;
		I2C_SCL_H;
		i2c_delay();
		if(I2C_SDA_GET)
		{
			vaule++;
		}
		I2C_SCL_L;
		i2c_delay();
	}
	
	SDA_OUT();
	return vaule;
}
