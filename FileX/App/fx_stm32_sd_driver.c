/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fx_stm32_sd_driver.c
  * @author  Application Team
  * @brief   FileX SD 卡底层驱动 (STM32H7 HAL_SD 轮询模式)
  *
  *          该驱动将 FileX 的介质读写请求映射到 STM32H7 的 SDMMC1 (HAL_SD),
  *          采用中断(IT)方式读写 + ThreadX 信号量等待完成。
  *          轮询方式在 RTOS 下会被其他任务抢占,30MHz 时 SDMMC FIFO 只有
  *          约 17us 的排空窗口,实测会出现 RX_OVERRUN 读失败;
  *          IT 模式由优先级最高的 SDMMC1 中断排空 FIFO,不再依赖线程调度。
  *
  *          分区支持:若卡 0 扇区是 MBR 且存在 FAT 分区,驱动自动把 FileX
  *          卷映射到该分区(引导扇区读写、数据读写均加分区偏移),
  *          从而直接打开卡上已有的 FAT32 文件系统,避免全卡重新格式化。
  *
  *          卡死防护:SDMMC_SWDATATIMEOUT 已在 stm32h7xx_hal_conf.h 中
  *          从 0xFFFFFFFF(约 49.7 天)改为 5000ms,HAL 内部读 SCR/卡状态
  *          超时后会返回错误,不再无限挂起;INIT 带 3 次重试。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fx_stm32_sd_driver.h"
#include "sdmmc.h"
#include "tx_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* 单次块读写超时时间 (ms) */
#define FX_SD_TIMEOUT                    30000U
/* 对应 tick 数(本项目 TX_TIMER_TICKS_PER_SECOND=1000,1 tick=1ms) */
#define FX_SD_TIMEOUT_TICKS              (FX_SD_TIMEOUT * TX_TIMER_TICKS_PER_SECOND / 1000U)

/* 卡初始化重试次数与间隔 */
#define FX_SD_INIT_RETRY                 3U
#define FX_SD_INIT_RETRY_DELAY_MS        100U

/* MBR 分区表偏移 */
#define FX_SD_MBR_PARTITION_OFFSET       446U
#define FX_SD_PARTITION_ENTRY_SIZE       16U
#define FX_SD_PARTITION_TYPE_OFFSET      4U
#define FX_SD_PARTITION_LBA_OFFSET       8U
#define FX_SD_PARTITION_SECTORS_OFFSET   12U

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static UCHAR  sd_mbr_sector[512U]     __attribute__((aligned(32)));
static UCHAR  sd_partition_boot[512U] __attribute__((aligned(32)));
static ULONG  sd_partition_start   = 0UL;
static ULONG  sd_partition_sectors = 0UL;
static UCHAR  sd_partition_mode    = 0U;
static TX_SEMAPHORE  sd_io_sem;
static volatile UINT sd_io_error   = 0U;
static UCHAR  sd_io_sem_ready      = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static ULONG  sd_read32(const UCHAR *p);
static UCHAR  sd_is_fat_boot(const UCHAR *sector);
static UCHAR  sd_is_fat_partition_type(UCHAR type);
static ULONG  sd_physical_sector(FX_MEDIA *media_ptr, ULONG logical_sector);
static void   sd_detect_partition(void);
static UINT   sd_raw_read_block(ULONG sector, UCHAR *buffer);
static UINT   sd_read_blocks(FX_MEDIA *media_ptr);
static UINT   sd_write_blocks(FX_MEDIA *media_ptr);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* FileX SD 驱动使用的中断完成回调(HAL 弱函数,在此强定义) */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  if (hsd == &hsd1)
  {
    sd_io_error = 0U;
    (void)tx_semaphore_put(&sd_io_sem);
  }
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  if (hsd == &hsd1)
  {
    sd_io_error = 0U;
    (void)tx_semaphore_put(&sd_io_sem);
  }
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  if (hsd == &hsd1)
  {
    sd_io_error = 1U;
    (void)tx_semaphore_put(&sd_io_sem);
  }
}

/* 等待一次 IT 传输完成;成功返回 FX_SUCCESS */
static UINT sd_io_wait_complete(void)
{
  UINT status = tx_semaphore_get(&sd_io_sem, FX_SD_TIMEOUT_TICKS);

  if (status != TX_SUCCESS)
  {
    /* 超时:中止传输并复位状态,避免卡死 */
    (void)HAL_SD_Abort(&hsd1);
    hsd1.State = HAL_SD_STATE_READY;
    return FX_IO_ERROR;
  }
  return (sd_io_error == 0U) ? FX_SUCCESS : FX_IO_ERROR;
}

/* 清掉可能残留的信号量计数,并开始一次新的 IT 传输 */
static void sd_io_start(void)
{
  while (tx_semaphore_get(&sd_io_sem, TX_NO_WAIT) == TX_SUCCESS)
  {
  }
  sd_io_error = 0U;
}

/* 读取单个物理扇区(供 MBR/分区识别使用) */
static UINT sd_raw_read_block(ULONG sector, UCHAR *buffer)
{
  UINT status;

  sd_io_start();
  if (HAL_SD_ReadBlocks_IT(&hsd1, buffer, sector, 1U) != HAL_OK)
  {
    printf("[ERR ] SD IT read start fail sector=%lu state=%u hal=0x%08lX\r\n",
           (unsigned long)sector, (unsigned int)hsd1.State, (unsigned long)hsd1.ErrorCode);
    return FX_IO_ERROR;
  }
  status = sd_io_wait_complete();
  if (status != FX_SUCCESS)
  {
    printf("[ERR ] SD raw read fail sector=%lu hal=0x%08lX\r\n",
           (unsigned long)sector, (unsigned long)hsd1.ErrorCode);
  }
  return status;
}

/* FileX 介质块读(IT 模式) */
static UINT sd_read_blocks(FX_MEDIA *media_ptr)
{
  ULONG sector = sd_physical_sector(media_ptr, media_ptr->fx_media_driver_logical_sector);
  UINT status;

  sd_io_start();
  if (HAL_SD_ReadBlocks_IT(&hsd1, media_ptr->fx_media_driver_buffer,
                           sector, media_ptr->fx_media_driver_sectors) != HAL_OK)
  {
    printf("[ERR ] SD IT read start fail req=%u sector=%lu state=%u hal=0x%08lX\r\n",
           (unsigned int)media_ptr->fx_media_driver_request, (unsigned long)sector,
           (unsigned int)hsd1.State, (unsigned long)hsd1.ErrorCode);
    return FX_IO_ERROR;
  }
  status = sd_io_wait_complete();
  if (status != FX_SUCCESS)
  {
    printf("[ERR ] SD read fail req=%u sector=%lu count=%lu hal=0x%08lX state=%u\r\n",
           (unsigned int)media_ptr->fx_media_driver_request, (unsigned long)sector,
           (unsigned long)media_ptr->fx_media_driver_sectors,
           (unsigned long)hsd1.ErrorCode, (unsigned int)hsd1.State);
  }
  return status;
}

/* FileX 介质块写(IT 模式);DATAEND 后还需等卡编程完成(BUSY 结束) */
static UINT sd_write_blocks(FX_MEDIA *media_ptr)
{
  ULONG sector = sd_physical_sector(media_ptr, media_ptr->fx_media_driver_logical_sector);
  UINT status;
  ULONG tickstart;

  sd_io_start();
  if (HAL_SD_WriteBlocks_IT(&hsd1, media_ptr->fx_media_driver_buffer,
                            sector, media_ptr->fx_media_driver_sectors) != HAL_OK)
  {
    printf("[ERR ] SD IT write start fail req=%u sector=%lu state=%u hal=0x%08lX\r\n",
           (unsigned int)media_ptr->fx_media_driver_request, (unsigned long)sector,
           (unsigned int)hsd1.State, (unsigned long)hsd1.ErrorCode);
    return FX_IO_ERROR;
  }
  status = sd_io_wait_complete();
  if (status != FX_SUCCESS)
  {
    printf("[ERR ] SD write fail req=%u sector=%lu count=%lu hal=0x%08lX state=%u\r\n",
           (unsigned int)media_ptr->fx_media_driver_request, (unsigned long)sector,
           (unsigned long)media_ptr->fx_media_driver_sectors,
           (unsigned long)hsd1.ErrorCode, (unsigned int)hsd1.State);
    return status;
  }

  /* 等待卡内部编程完成,避免连续写时返回 BUSY */
  tickstart = HAL_GetTick();
  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
    if ((HAL_GetTick() - tickstart) >= FX_SD_TIMEOUT)
    {
      printf("[ERR ] SD card busy timeout after write, sector=%lu\r\n", (unsigned long)sector);
      return FX_IO_ERROR;
    }
  }
  return FX_SUCCESS;
}

static ULONG sd_read32(const UCHAR *p)
{
  return ((ULONG)p[0]) | (((ULONG)p[1]) << 8) | (((ULONG)p[2]) << 16) | (((ULONG)p[3]) << 24);
}

/* 判断一个 512 字节扇区是否为 FAT 引导扇区(FAT12/16/32) */
static UCHAR sd_is_fat_boot(const UCHAR *sector)
{
  if (!((sector[0] == 0xE9U) || ((sector[0] == 0xEBU) && (sector[2] == 0x90U))))
  {
    return 0U;
  }
  /* FAT12/16 的 "FAT12   " 位于 0x36;FAT32 的 "FAT32   " 位于 0x52 */
  if ((sector[0x36] == 'F') && (sector[0x37] == 'A') && (sector[0x38] == 'T'))
  {
    return 1U;
  }
  if ((sector[0x52] == 'F') && (sector[0x53] == 'A') && (sector[0x54] == 'T'))
  {
    return 1U;
  }
  return 0U;
}

/* FAT12/16/32 分区类型 */
static UCHAR sd_is_fat_partition_type(UCHAR type)
{
  return (type == 0x01U) || (type == 0x04U) || (type == 0x06U) ||
         (type == 0x0BU) || (type == 0x0CU) || (type == 0x0EU);
}

/* 计算 FileX 逻辑扇区对应的物理扇区:
   - 无分区(整卡卷):logical + hidden
   - 有分区且 BPB hidden=0(如本驱动格式化):logical + 分区偏移
   - 有分区且 BPB hidden=分区偏移(Windows 格式化):logical + hidden */
static ULONG sd_physical_sector(FX_MEDIA *media_ptr, ULONG logical_sector)
{
  ULONG physical = logical_sector + media_ptr->fx_media_hidden_sectors;

  if (sd_partition_mode && (media_ptr->fx_media_hidden_sectors == 0UL))
  {
    physical += sd_partition_start;
  }
  return physical;
}

/* 读取 0 扇区并识别 MBR/FAT 分区(仅识别 FAT12/16/32 主分区) */
static void sd_detect_partition(void)
{
  UINT i;

  sd_partition_start   = 0UL;
  sd_partition_sectors = 0UL;
  sd_partition_mode    = 0U;

  if (sd_raw_read_block(0UL, sd_mbr_sector) != FX_SUCCESS)
  {
    printf("[WARN] SD: read sector0 failed, use raw volume mode\r\n");
    return;
  }

  /* 简要打印 0 扇区特征,便于诊断卡内容 */
  printf("[INFO] SD: sector0 jump=%02X %02X %02X, sig=%02X%02X, FAT16='%c%c%c', FAT32='%c%c%c'\r\n",
         (unsigned int)sd_mbr_sector[0], (unsigned int)sd_mbr_sector[1], (unsigned int)sd_mbr_sector[2],
         (unsigned int)sd_mbr_sector[511], (unsigned int)sd_mbr_sector[510],
         (sd_mbr_sector[0x36] >= 0x20 && sd_mbr_sector[0x36] < 0x7F) ? (char)sd_mbr_sector[0x36] : '.',
         (sd_mbr_sector[0x37] >= 0x20 && sd_mbr_sector[0x37] < 0x7F) ? (char)sd_mbr_sector[0x37] : '.',
         (sd_mbr_sector[0x38] >= 0x20 && sd_mbr_sector[0x38] < 0x7F) ? (char)sd_mbr_sector[0x38] : '.',
         (sd_mbr_sector[0x52] >= 0x20 && sd_mbr_sector[0x52] < 0x7F) ? (char)sd_mbr_sector[0x52] : '.',
         (sd_mbr_sector[0x53] >= 0x20 && sd_mbr_sector[0x53] < 0x7F) ? (char)sd_mbr_sector[0x53] : '.',
         (sd_mbr_sector[0x54] >= 0x20 && sd_mbr_sector[0x54] < 0x7F) ? (char)sd_mbr_sector[0x54] : '.');

  /* 0 扇区本身就是 FAT 引导扇区:整卡卷,无需分区偏移 */
  if (sd_is_fat_boot(sd_mbr_sector))
  {
    printf("[INFO] SD: sector0 is FAT boot sector (raw volume)\r\n");
    return;
  }

  if ((sd_mbr_sector[510] != 0x55U) || (sd_mbr_sector[511] != 0xAAU))
  {
    printf("[INFO] SD: no MBR signature, raw volume mode\r\n");
    return;
  }

  /* 遍历 4 个主分区表项 */
  for (i = 0U; i < 4U; i++)
  {
    const UCHAR *entry = &sd_mbr_sector[FX_SD_MBR_PARTITION_OFFSET + i * FX_SD_PARTITION_ENTRY_SIZE];
    UCHAR type  = entry[FX_SD_PARTITION_TYPE_OFFSET];
    ULONG start = sd_read32(&entry[FX_SD_PARTITION_LBA_OFFSET]);
    ULONG size  = sd_read32(&entry[FX_SD_PARTITION_SECTORS_OFFSET]);

    if (!sd_is_fat_partition_type(type) || (start == 0UL) || (size == 0UL))
    {
      continue;
    }

    printf("[INFO] SD: MBR entry %u: type=0x%02X start=%lu size=%lu, checking boot sector...\r\n",
           (unsigned int)i, (unsigned int)type, (unsigned long)start, (unsigned long)size);

    /* 读取分区引导扇区,确认确实是 FAT BPB,避免误判 */
    if ((sd_raw_read_block(start, sd_partition_boot) == FX_SUCCESS) &&
        sd_is_fat_boot(sd_partition_boot))
    {
      sd_partition_start   = start;
      sd_partition_sectors = size;
      sd_partition_mode    = 1U;
      printf("[OK ] SD: FAT partition found: type=0x%02X, start LBA=%lu, sectors=%lu\r\n",
             (unsigned int)type, (unsigned long)start, (unsigned long)size);
      return;
    }

    printf("[WARN] SD: partition %u type=0x%02X start=%lu size=%lu, boot sector not FAT, skip\r\n",
           (unsigned int)i, (unsigned int)type, (unsigned long)start, (unsigned long)size);
  }

  printf("[INFO] SD: MBR present but no usable FAT partition, raw volume mode\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  FileX 介质驱动入口,由 FileX 内部调用
  * @param  media_ptr: FileX 介质控制块指针
  * @retval 无(结果通过 fx_media_driver_status 返回)
  */
VOID fx_stm32_sd_driver(FX_MEDIA *media_ptr)
{
  UINT status = FX_SUCCESS;

  /* USER CODE BEGIN fx_stm32_sd_driver */

  switch (media_ptr->fx_media_driver_request)
  {
    case FX_DRIVER_INIT:
    {
      UINT attempt;

      /* 首次使用时创建 IT 完成信号量(在 FileX 线程上下文执行) */
      if (!sd_io_sem_ready)
      {
        if (tx_semaphore_create(&sd_io_sem, "sd_io_sem", 0U) != TX_SUCCESS)
        {
          status = FX_IO_ERROR;
          break;
        }
        sd_io_sem_ready = 1U;
      }

      /* 卡未就绪时初始化(带重试)。SDMMC_SWDATATIMEOUT 已限制为 5 秒,
         卡异常时 HAL_SD_Init 会超时返回,不会再无限挂起。 */
      for (attempt = 0U; attempt < FX_SD_INIT_RETRY; attempt++)
      {
        if (hsd1.State == HAL_SD_STATE_READY)
        {
          break;
        }
        hsd1.State     = HAL_SD_STATE_RESET;
        hsd1.ErrorCode = HAL_SD_ERROR_NONE;
        if (HAL_SD_Init(&hsd1) == HAL_OK)
        {
          break;
        }
        HAL_Delay(FX_SD_INIT_RETRY_DELAY_MS);
      }

      if (hsd1.State != HAL_SD_STATE_READY)
      {
        printf("[ERR ] SD init failed after %u attempts (err=0x%08lX)\r\n",
               (unsigned int)FX_SD_INIT_RETRY, (unsigned long)hsd1.ErrorCode);
        status = FX_IO_ERROR;
        break;
      }

      /* 确保 4 位总线模式 */
      if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
      {
        status = FX_IO_ERROR;
        break;
      }

      /* 识别 MBR/FAT 分区,决定是否加分区偏移 */
      sd_detect_partition();
      break;
    }

    case FX_DRIVER_UNINIT:
    {
      /* 不真正 DeInit SD 卡:卡保持激活,避免后续重复 HAL_SD_Init 卡死 */
      status = FX_SUCCESS;
      break;
    }

    case FX_DRIVER_READ:
    case FX_DRIVER_BOOT_READ:
    {
      if (sd_read_blocks(media_ptr) != FX_SUCCESS)
      {
        status = FX_IO_ERROR;
      }
      break;
    }

    case FX_DRIVER_WRITE:
    case FX_DRIVER_BOOT_WRITE:
    {
      if (sd_write_blocks(media_ptr) != FX_SUCCESS)
      {
        status = FX_IO_ERROR;
      }
      break;
    }

    case FX_DRIVER_FLUSH:
    case FX_DRIVER_ABORT:
    case FX_DRIVER_RELEASE_SECTORS:
    {
      /* SD 卡驱动无需额外处理 */
      status = FX_SUCCESS;
      break;
    }

    default:
    {
      status = FX_IO_ERROR;
      break;
    }
  }

  /* USER CODE END fx_stm32_sd_driver */

  /* 将执行结果返回给 FileX */
  media_ptr->fx_media_driver_status = status;
}

/* USER CODE BEGIN 1 */

ULONG fx_stm32_sd_partition_start(void)
{
  return sd_partition_start;
}

ULONG fx_stm32_sd_partition_sectors(void)
{
  return sd_partition_sectors;
}

/* USER CODE END 1 */
