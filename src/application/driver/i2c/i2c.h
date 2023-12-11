#ifndef __I2C_H
#define __I2C_H

#include "stm32f4xx_hal.h"

#define SDA_IN()  {GPIOB->MODER&=~(3<<(8*2)); GPIOB->MODER |= (0<<(8*2));} /* PB8 input mode */
#define SDA_OUT() {GPIOB->MODER&=~(3<<(8*2)); GPIOB->MODER |= (1<<(8*2));} /* PB8 output mode */

#define I2C_SCL_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define I2C_SCL_L    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define I2C_SDA_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define I2C_SDA_L    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define I2C_SDA_GET  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)


void delay_ms(uint16_t ms);

void i2c_init(void);
void i2c_send_byte(uint8_t SendByte);
uint8_t i2c_receive_byte(void);

#endif
