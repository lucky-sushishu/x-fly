
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_filex.c
  * @author  MCD Application Team
  * @brief   FileX applicative file
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
#include "app_filex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sdmmc.h"
#include "fx_stm32_sd_driver.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* Main thread stack size */
#define FX_APP_THREAD_STACK_SIZE         8192
/* Main thread priority */
#define FX_APP_THREAD_PRIO               10

/* USER CODE BEGIN PD */
/* FileX 介质缓存大小(字节),越大缓存扇区越多,读写越快 */
#define FX_MEDIA_MEMORY_SIZE             (32U * 512U)

/* 测试文件相关参数 */
#define FX_DEMO_FILE_NAME                "DEMO.TXT"
#define FX_DEMO_DATA_SIZE                512U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Main thread global data structures.  */
TX_THREAD       fx_app_thread;

/* USER CODE BEGIN PV */
/* FileX 介质控制块 */
FX_MEDIA        sd_disk;
/* 测试用文件控制块 */
FX_FILE         sd_file;
/* FileX 介质缓存(扇区缓存),32 字节对齐,方便将来升级为 DMA 方式 */
static UCHAR    fx_media_memory[FX_MEDIA_MEMORY_SIZE] __attribute__((aligned(32)));
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* Main thread entry function.  */
void fx_app_thread_entry(ULONG thread_input);

/* USER CODE BEGIN PFP */
static void fx_demo_print_media_info(void);
static void fx_demo_file_test(void);
static void fx_demo_list_dir(void);
/* USER CODE END PFP */

/**
  * @brief  Application FileX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_FileX_Init(VOID *memory_ptr)
{
  UINT ret = FX_SUCCESS;

  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  VOID *pointer;

  /* USER CODE BEGIN MX_FileX_MEM_POOL */

  /* USER CODE END MX_FileX_MEM_POOL */

  /* USER CODE BEGIN 0 */

  /* USER CODE END 0 */

  /*Allocate memory for the main thread's stack*/
  ret = tx_byte_allocate(byte_pool, &pointer, FX_APP_THREAD_STACK_SIZE, TX_NO_WAIT);

  /* Check FX_APP_THREAD_STACK_SIZE allocation*/
  if (ret != FX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the main thread.  */
  ret = tx_thread_create(&fx_app_thread, FX_APP_THREAD_NAME, fx_app_thread_entry, 0, pointer, FX_APP_THREAD_STACK_SIZE,
                         FX_APP_THREAD_PRIO, FX_APP_PREEMPTION_THRESHOLD, FX_APP_THREAD_TIME_SLICE, FX_APP_THREAD_AUTO_START);

  /* Check main thread creation */
  if (ret != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }
  /* USER CODE BEGIN MX_FileX_Init */

  /* USER CODE END MX_FileX_Init */

  /* Initialize FileX.  */
  fx_system_initialize();

  /* USER CODE BEGIN MX_FileX_Init 1*/

  /* USER CODE END MX_FileX_Init 1*/

  return ret;
}

 /**
 * @brief  Main thread entry.
 * @param thread_input: ULONG user argument used by the thread entry
 * @retval none
 */
void fx_app_thread_entry(ULONG thread_input)
{
  /* USER CODE BEGIN fx_app_thread_entry 0 */
  UINT status;
  UINT sectors_per_cluster;
  ULONG format_sectors;
  ULONG format_t0;
  HAL_SD_CardInfoTypeDef card_info;
  /* USER CODE END fx_app_thread_entry 0 */

  /* USER CODE BEGIN fx_app_thread_entry 1 */

  printf("\r\n========== FileX + SD Card Demo ==========\r\n");

  /* 1. 读取 SD 卡基本信息 */
  memset(&card_info, 0, sizeof(card_info));
  if (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK)
  {
    printf("[ERR] HAL_SD_GetCardInfo failed! check SD card & wiring\r\n");
  }
  else
  {
    printf("[OK ] SD card info:\r\n");
    printf("     CardType    : 0x%08lX\r\n", (unsigned long)card_info.CardType);
    printf("     BlockNbr    : %lu (%.1f MB)\r\n", (unsigned long)card_info.BlockNbr,
           (double)card_info.BlockNbr * card_info.BlockSize / 1024.0 / 1024.0);
    printf("     LogBlockNbr : %lu\r\n", (unsigned long)card_info.LogBlockNbr);
    printf("     LogBlockSize: %lu\r\n", (unsigned long)card_info.LogBlockSize);
  }

  /* 2. 打开介质(会触发驱动 FX_DRIVER_INIT,完成卡初始化并识别 MBR 分区) */
  printf("[INFO] fx_media_open ...\r\n");
  status = fx_media_open(&sd_disk, "SD_DISK", fx_stm32_sd_driver, (VOID *)&hsd1,
                         fx_media_memory, FX_MEDIA_MEMORY_SIZE);
  if (status != FX_SUCCESS)
  {
    /* 打开失败:可能是未格式化的卡,先格式化再打开 */
    printf("[WARN] fx_media_open failed (0x%02X), try format...\r\n", (unsigned int)status);

    /* 根据容量选择簇大小,保证大容量卡格式化为 FAT32 */
    if (card_info.LogBlockNbr >= 4194304UL)   /* >= 2GB */
    {
      sectors_per_cluster = 64U;              /* 32KB 簇 */
    }
    else
    {
      sectors_per_cluster = 8U;               /* 4KB 簇 */
    }

    /* 若卡上有 MBR FAT 分区,只格式化该分区,保留 MBR 与其余分区;
       否则格式化整卡 */
    format_sectors = card_info.LogBlockNbr;
    if (fx_stm32_sd_partition_start() != 0UL)
    {
      format_sectors = fx_stm32_sd_partition_sectors();
      printf("[INFO] formatting existing FAT partition (%lu sectors)...\r\n",
             (unsigned long)format_sectors);
    }
    else
    {
      printf("[INFO] formatting whole card (%lu sectors), 30GB 卡约需 5~10 分钟,请勿断电...\r\n",
             (unsigned long)format_sectors);
    }

    format_t0 = HAL_GetTick();
    status = fx_media_format(&sd_disk, fx_stm32_sd_driver, (VOID *)&hsd1,
                             fx_media_memory, FX_MEDIA_MEMORY_SIZE,
                             "X-FLY", 1U, 512U, 0U,
                             format_sectors, card_info.LogBlockSize,
                             sectors_per_cluster, 0xFFU, 0x3FU);
    if (status != FX_SUCCESS)
    {
      printf("[ERR ] fx_media_format failed (0x%02X)\r\n", (unsigned int)status);
      goto demo_fail;
    }
    printf("[OK ] format done in %lu s, reopening...\r\n",
           (unsigned long)((HAL_GetTick() - format_t0) / 1000UL));

    status = fx_media_open(&sd_disk, "SD_DISK", fx_stm32_sd_driver, (VOID *)&hsd1,
                           fx_media_memory, FX_MEDIA_MEMORY_SIZE);
    if (status != FX_SUCCESS)
    {
      printf("[ERR ] fx_media_open after format failed (0x%02X)\r\n", (unsigned int)status);
      goto demo_fail;
    }
  }
  printf("[OK ] fx_media_open success\r\n");
  fx_demo_print_media_info();

  /* 3. 文件读写测试 */
  fx_demo_file_test();

  /* 4. 目录列表 */
  fx_demo_list_dir();

  /* 5. 周期打印剩余空间 */
  while (1)
  {
    ULONG64 free_bytes = 0;

    tx_thread_sleep(5000);

    /* 定期刷盘,任意时刻拔卡/断电也不丢数据 */
    (void)fx_media_flush(&sd_disk);

    status = fx_media_extended_space_available(&sd_disk, &free_bytes);
    if (status == FX_SUCCESS)
    {
      /* newlib-nano 不支持 %llu,这里按 MB 输出 */
      printf("[INFO] free space: %lu MB\r\n", (unsigned long)(free_bytes >> 20));
    }
  }

demo_fail:
  printf("========== FileX SD Demo FAILED ==========\r\n");
  while (1)
  {
    tx_thread_sleep(10000);
  }

  /* USER CODE END fx_app_thread_entry 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief  打印 FileX 介质信息
  */
static void fx_demo_print_media_info(void)
{
  printf("media info:\r\n");
  printf("     total sectors   : %lu\r\n", (unsigned long)sd_disk.fx_media_total_sectors);
  printf("     bytes/sector    : %u\r\n", (unsigned int)sd_disk.fx_media_bytes_per_sector);
  printf("     total clusters  : %lu\r\n", (unsigned long)sd_disk.fx_media_total_clusters);
  printf("     sectors/cluster : %u\r\n", (unsigned int)sd_disk.fx_media_sectors_per_cluster);
  printf("     FAT type        : %s\r\n",
         sd_disk.fx_media_32_bit_FAT ? "FAT32" :
         (sd_disk.fx_media_12_bit_FAT ? "FAT12" : "FAT16"));
}

/**
  * @brief  文件创建 / 写入 / 读回校验测试
  */
static void fx_demo_file_test(void)
{
  UINT status;
  UINT i;
  UCHAR write_buf[FX_DEMO_DATA_SIZE];
  UCHAR read_buf[FX_DEMO_DATA_SIZE];
  ULONG actual_size = 0;
  UINT mismatch = 0;

  /* 参考数据:0..255 循环 */
  for (i = 0; i < FX_DEMO_DATA_SIZE; i++)
  {
    write_buf[i] = (UCHAR)(i & 0xFFU);
  }

  /* 若文件已存在且内容正确,说明上次写入已成功落盘,直接验证,避免反复重写 */
  status = fx_file_open(&sd_disk, &sd_file, FX_DEMO_FILE_NAME, FX_OPEN_FOR_READ);
  if (status == FX_SUCCESS)
  {
    for (i = 0U; i < 4U; i++)
    {
      status = fx_file_read(&sd_file, read_buf, FX_DEMO_DATA_SIZE, &actual_size);
      if ((status != FX_SUCCESS) || (actual_size != FX_DEMO_DATA_SIZE) ||
          (memcmp(write_buf, read_buf, FX_DEMO_DATA_SIZE) != 0))
      {
        mismatch++;
      }
    }
    fx_file_close(&sd_file);
    if (mismatch == 0U)
    {
      printf("[OK ] %s exists, verify PASSED (persisted on card)\r\n", FX_DEMO_FILE_NAME);
      return;
    }
    printf("[INFO] %s content mismatch, rewrite...\r\n", FX_DEMO_FILE_NAME);
    (void)fx_file_delete(&sd_disk, FX_DEMO_FILE_NAME);
  }
  else
  {
    printf("[INFO] %s not found, create & write...\r\n", FX_DEMO_FILE_NAME);
  }

  /* 创建文件 */
  status = fx_file_create(&sd_disk, FX_DEMO_FILE_NAME);
  if (status != FX_SUCCESS)
  {
    printf("[ERR ] fx_file_create failed: 0x%02X\r\n", (unsigned int)status);
    return;
  }

  /* 以写方式打开 */
  status = fx_file_open(&sd_disk, &sd_file, FX_DEMO_FILE_NAME, FX_OPEN_FOR_WRITE);
  if (status != FX_SUCCESS)
  {
    printf("[ERR ] fx_file_open(write) failed: 0x%02X\r\n", (unsigned int)status);
    return;
  }

  /* 写入 4 次 512 字节 = 2KB 测试数据 */
  for (i = 0; i < 4U; i++)
  {
    status = fx_file_write(&sd_file, write_buf, FX_DEMO_DATA_SIZE);
    if (status != FX_SUCCESS)
    {
      printf("[ERR ] fx_file_write[%u] failed: 0x%02X\r\n", (unsigned int)i, (unsigned int)status);
      fx_file_close(&sd_file);
      return;
    }
  }

  /* 关闭文件 */
  status = fx_file_close(&sd_file);
  if (status != FX_SUCCESS)
  {
    printf("[ERR ] fx_file_close failed: 0x%02X\r\n", (unsigned int)status);
    return;
  }
  printf("[OK ] write 2KB to %s done\r\n", FX_DEMO_FILE_NAME);

  /* 关键:FileX 写操作先进入 RAM 扇区缓存,fx_file_close 不会把数据立即全部落盘,
     必须调用 fx_media_flush 刷到 SD 卡,否则拔卡/断电会丢数据 */
  status = fx_media_flush(&sd_disk);
  if (status != FX_SUCCESS)
  {
    printf("[ERR ] fx_media_flush failed: 0x%02X\r\n", (unsigned int)status);
    return;
  }
  printf("[OK ] media flushed to SD card\r\n");

  /* 以读方式重新打开并读回校验 */
  status = fx_file_open(&sd_disk, &sd_file, FX_DEMO_FILE_NAME, FX_OPEN_FOR_READ);
  if (status != FX_SUCCESS)
  {
    printf("[ERR ] fx_file_open(read) failed: 0x%02X\r\n", (unsigned int)status);
    return;
  }

  for (i = 0; i < 4U; i++)
  {
    status = fx_file_read(&sd_file, read_buf, FX_DEMO_DATA_SIZE, &actual_size);
    if ((status != FX_SUCCESS) || (actual_size != FX_DEMO_DATA_SIZE))
    {
      printf("[ERR ] fx_file_read[%u] failed: 0x%02X, actual=%lu\r\n",
             (unsigned int)i, (unsigned int)status, (unsigned long)actual_size);
      break;
    }
    if (memcmp(write_buf, read_buf, FX_DEMO_DATA_SIZE) != 0)
    {
      mismatch++;
    }
  }

  fx_file_close(&sd_file);

  if (mismatch == 0U)
  {
    printf("[OK ] read back & verify 2KB PASSED (data consistent)\r\n");
  }
  else
  {
    printf("[ERR] read back mismatch count = %u\r\n", (unsigned int)mismatch);
  }
}

/**
  * @brief  列出根目录所有文件
  */
static void fx_demo_list_dir(void)
{
  UINT status;
  CHAR name[FX_MAX_LONG_NAME_LEN];

  printf("root dir listing:\r\n");
  status = fx_directory_first_entry_find(&sd_disk, name);
  if (status == FX_SUCCESS)
  {
    do
    {
      printf("     [%s]\r\n", name);
      status = fx_directory_next_entry_find(&sd_disk, name);
    } while (status == FX_SUCCESS);
  }
  else
  {
    printf("     (empty)\r\n");
  }
}

/* USER CODE END 1 */
