/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "gpio.h"
/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN Private defines */
typedef enum spi_cs_e spi_cs_t;

enum spi_cs_e
{
	SPI3_CS1,
	SPI3_CS2,
	SPI3_CS3,
	SPI3_CS4,
};

#define SPI3_CS1_ENABLE()		HAL_GPIO_WritePin(SPI3_CS1_GPIO_Port, SPI3_CS1_Pin, GPIO_PIN_RESET)
#define SPI3_CS1_DISABLE()		HAL_GPIO_WritePin(SPI3_CS1_GPIO_Port, SPI3_CS1_Pin, GPIO_PIN_SET)
#define SPI3_CS2_ENABLE()		HAL_GPIO_WritePin(SPI3_CS2_GPIO_Port, SPI3_CS2_Pin, GPIO_PIN_RESET)
#define SPI3_CS2_DISABLE()		HAL_GPIO_WritePin(SPI3_CS2_GPIO_Port, SPI3_CS2_Pin, GPIO_PIN_SET)
#define SPI3_CS3_ENABLE()		HAL_GPIO_WritePin(SPI3_CS3_GPIO_Port, SPI3_CS3_Pin, GPIO_PIN_RESET)
#define SPI3_CS3_DISABLE()		HAL_GPIO_WritePin(SPI3_CS3_GPIO_Port, SPI3_CS3_Pin, GPIO_PIN_SET)
#define SPI3_CS4_ENABLE()		HAL_GPIO_WritePin(SPI3_CS4_GPIO_Port, SPI3_CS4_Pin, GPIO_PIN_RESET)
#define SPI3_CS4_DISABLE()		HAL_GPIO_WritePin(SPI3_CS4_GPIO_Port, SPI3_CS4_Pin, GPIO_PIN_SET)

/* USER CODE END Private defines */

void MX_SPI3_Init(void);

/* USER CODE BEGIN Prototypes */
void spi_init(void);
void spi_enable_cs(spi_cs_t cs);
void spi_disable_cs(spi_cs_t cs);
int spi_read(spi_cs_t cs, uint8_t *data,  uint8_t data_length);
int spi_write(spi_cs_t cs, const uint8_t *data,  uint8_t data_length);
int spi_write_read(spi_cs_t cs, uint8_t *tx_data, uint8_t *rx_data, uint16_t tx_length);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

