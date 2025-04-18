#ifndef _BSP_SFTWARE_I2C_H
#define _BSP_SFTWARE_I2C_H

#include "stm32f4xx_hal.h"
#include "bsp_delay.h"

#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2)); GPIOB->MODER |= (0<<(9*2));} /* PB9 input mode */
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2)); GPIOB->MODER |= (1<<(9*2));} /* PB9 output mode */

#define I2C_SCL_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define I2C_SCL_L    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define I2C_SDA_H    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define I2C_SDA_L    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define I2C_SDA_GET  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)


void delay_ms(uint16_t ms);

void i2c_init(void);

void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_wait_ack(void);
void i2c_ack(void);
void i2c_nack(void);


void i2c_send(uint8_t SendByte);
uint8_t i2c_read(uint8_t ack);

uint8_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t i2c_read_byte(uint8_t addr, uint8_t reg);
uint8_t i2c_read_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t i2c_write_len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
