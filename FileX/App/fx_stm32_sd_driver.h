/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fx_stm32_sd_driver.h
  * @author  Application Team
  * @brief   FileX SD 卡底层驱动头文件 (STM32H7 HAL_SD 轮询模式)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __FX_STM32_SD_DRIVER_H__
#define __FX_STM32_SD_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "fx_api.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported functions prototypes ---------------------------------------------*/
VOID fx_stm32_sd_driver(FX_MEDIA *media_ptr);

/* 若介质 0 扇区为 MBR 且存在可用的 FAT 分区,返回分区起始 LBA / 扇区数;
   否则返回 0。用于 demo 选择格式化范围(格式化分区而不是整卡)。 */
ULONG fx_stm32_sd_partition_start(void);
ULONG fx_stm32_sd_partition_sectors(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __FX_STM32_SD_DRIVER_H__ */
