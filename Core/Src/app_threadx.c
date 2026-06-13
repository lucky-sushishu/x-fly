/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "spi.h"
#include "i2c.h"
#include "bmi088.h"
#include "icm20602.h"
#include "icm20689.h"
#include "ist8310.h"
#include "ms5611.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define BMI088_TEST
//#define ICM20602_TEST
//#define ICM20689_TEST
//#define IST8310_TEST
#define MS5611_TEST
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TX_THREAD tx_app_thread;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  /* USER CODE END App_ThreadX_MEM_POOL */
  CHAR *pointer;

  /* Allocate the stack for tx app thread  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       TX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  /* Create tx app thread.  */
  if (tx_thread_create(&tx_app_thread, "tx app thread", tx_app_thread_entry, 0, pointer,
                       TX_APP_STACK_SIZE, TX_APP_THREAD_PRIO, TX_APP_THREAD_PREEMPTION_THRESHOLD,
                       TX_APP_THREAD_TIME_SLICE, TX_APP_THREAD_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  /* USER CODE BEGIN App_ThreadX_Init */

  /* USER CODE END App_ThreadX_Init */

  return ret;
}
/**
  * @brief  Function implementing the tx_app_thread_entry thread.
  * @param  thread_input: Hardcoded to 0.
  * @retval None
  */
void tx_app_thread_entry(ULONG thread_input)
{
  /* USER CODE BEGIN tx_app_thread_entry */
//	int i = 0;
#ifdef BMI088_TEST
	uint8_t bmi088_acce_chip_id = 0;
	uint8_t bmi088_gyro_chip_id = 0;
#endif
#ifdef ICM20602_TEST
	uint8_t icm20602_chip_id    = 0;
#endif
#ifdef ICM20689_TEST
	uint8_t icm20689_chip_id    = 0;
#endif

	LED_ERROR_CLR();
	spi_init();
#ifdef BMI088_TEST
	bmi088_write(BMI088_ACCE, BMI088_ACCE_PWR_CTRL, BMI088_ACC_PWR_CTRL_ACC_ENABLE);
	tx_thread_sleep(500);
#endif
#ifdef ICM20602_TEST
	icm20602_write(ICM20602_PWR_MGMT_1, 0x01);
	tx_thread_sleep(20);
#endif

#ifdef MS5611_TEST
	ms5611_init();
	tx_thread_sleep(100);
#endif

	printf("\r\n--------- --------- --------- ---------\r\n");

	while (1)
	{
#ifdef BMI088_TEST
		/* bmi088 test */
		bmi088_read(BMI088_ACCE, BMI088_ACCE_CHIP_ID, &bmi088_acce_chip_id);
		tx_thread_sleep(500);
		bmi088_read(BMI088_GYRO, BMI088_GYRO_CHIP_ID, &bmi088_gyro_chip_id);
		tx_thread_sleep(500);
		printf("bmi088: acce id 0x%02x, gyro id 0x%02x, %05d\r\n", bmi088_acce_chip_id, bmi088_gyro_chip_id, i++);
#endif


#ifdef ICM20602_TEST
		/* icm20602 test */
		icm20602_read(ICM20602_WHO_AM_I, &icm20602_chip_id);
		tx_thread_sleep(500);
		printf("icm2062: id 0x%02x, %05d\r\n", icm20602_chip_id, i++);
#endif


#ifdef ICM20689_TEST
		icm20689_read(ICM20689_WHO_AM_I, &icm20689_chip_id);
		tx_thread_sleep(500);
		printf("icm2089: id 0x%02x, %05d\r\n", icm20689_chip_id, i++);
#endif


#ifdef IST8310_TEST
		uint8_t ist8310_device_id = 0;
		ist8310_read_byte(IST8310_WHO_AM_I, &ist8310_device_id);
		printf("ist8310: id 0x%02x, %05d\r\n", ist8310_device_id, i++);
		tx_thread_sleep(500);
#endif


#ifdef MS5611_TEST
		uint8_t ms5611_value[3] = {0};
//		ms5611_read_pressure_value(&ms5611_value[0]);
//		tx_thread_sleep(100);
//		printf("ms5611: pressure value 0x%02x, 0x%02x, 0x%02x, %05d\r\n", ms5611_value[0], ms5611_value[1], ms5611_value[2], i++);

		ms5611_read_temperature_value(&ms5611_value[0]);
//		printf("ms5611: temperature value 0x%02x, 0x%02x, 0x%02x, %05d\r\n", ms5611_value[0], ms5611_value[1], ms5611_value[2], i++);
		uint32_t D2 	  = (uint32_t)ms5611_value[0] << 16 | (uint32_t)ms5611_value[1] << 8 | (uint32_t)ms5611_value[2];
		int32_t dT 	 	  = D2 - (g_ms5611_prom[4] * (1 <<8));
		int32_t TEMP 	  = 2000 + (dT * g_ms5611_prom[5] / (1 <<23));
		float temperature = (float)TEMP / 100.0f;
		printf("ms5611 temperature=%f\r\n", temperature);
		tx_thread_sleep(100);
#endif


#if 0
		uint8_t Address = 0;
		for(Address = 1; Address < 128; Address++)
		{
		    // 这里的 Address << 1 是因为 HAL 库通常需要 8 位写地址
		    if (HAL_I2C_IsDeviceReady(&hi2c1, (Address << 1), 1, 10) == HAL_OK)
		    {
		        printf("发现 I2C 设备，7位地址为: 0x%02X\r\n", Address);
		    }
		}
		tx_thread_sleep(500);
#endif
	}

  /* USER CODE END tx_app_thread_entry */
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
