# AI 代码修改记录

> 约定：
> 1. 本文件用于记录 AI 生成的代码改动，按提交（commit）次数编号（Commit 1、Commit 2……），随对应 commit 一起提交；
> 2. AI 提交的 commit message 带 `（AI）` 标记（中文括号），置于主题前部，例如：`FileX：（AI）SDMMC(SDIO) + FileX 驱动适配`；
> 3. 每条记录保存完整的 commit message（commit N 的完整描述），便于复盘时按提交描述检索；
> 4. 每条记录包含：改动内容、修改方案、测试过程、边界风险。

---

## Commit 1：SDMMC(SDIO) + FileX 驱动适配

**commit message：`FileX：（AI）SDMMC(SDIO) + FileX 驱动适配`**

- 日期：2026-08-02
- 范围：SD 卡底层驱动、FileX 应用 demo、相关 CubeMX/HAL 配置

### 改动内容

1. SDMMC1 从 MMC 模式改为 SD 模式（HAL_SD），同步 `x-fly.ioc`，防止 CubeMX 重新生成代码时回退；
2. 新增 FileX SD 底层驱动 `fx_stm32_sd_driver.c/.h`（中断模式 + ThreadX 信号量）；
3. 重写测试 demo（`app_filex.c`）：卡信息打印、介质打开、MBR/FAT 分区识别、文件写读校验、目录列表、剩余空间查询；
4. 稳定性与可靠性修复：
   - `SDMMC_SWDATATIMEOUT` 由 0xFFFFFFFF（约 49.7 天）改为 5000ms，杜绝 HAL 初始化无限等待；
   - 开启 SDMMC 硬件流控，解决 RTOS 下轮询排空 FIFO 导致的 `RX_OVERRUN`；
   - SDMMC GPIO 改为内部上拉，改善接触不良；
   - SD 初始化失败不再进 `Error_Handler`，由驱动重试，保证飞控不因 SD 卡死机；
   - 写文件后调用 `fx_media_flush` 真正落盘，避免拔卡/断电丢数据；
5. 删除旧的 `SD_FileX_适配工作记录.md`（内容精简后并入本文件）。

### 修改方案

- 卡识别：MMC 模式用 CMD1，SD 卡只响应 ACMD41，因此必须切换为 HAL_SD；
- 分区感知：驱动 INIT 时读 0 扇区，若为 MBR 且存在 FAT12/16/32 分区，则二次读取分区引导扇区校验为 FAT BPB 后，将 FileX 读写偏移到分区起始 LBA，直接打开卡上已有 FAT32 分区，避免 30GB 全卡格式化（约 5~10 分钟）并保留卡上数据；
- 传输方式：`HAL_SD_ReadBlocks_IT/WriteBlocks_IT` + 信号量等待（30s 超时）；写完成后再等待卡编程结束（`HAL_SD_GetCardState`），避免连续写返回 BUSY；
- 超时/落盘：HAL 内部软件超时改为 5s；demo 写完调用 `fx_media_flush`，周期循环每 5s 刷盘一次。

### 测试过程

- 编译：GNU Tools for STM32 13.3.rel1 完整编译链接通过（0 error / 0 warning）；
- 烧录：STM32_Programmer_CLI SWD 烧录 + 复位；
- 串口实测：
  - 卡识别正常：30GB SDXC（BlockNbr=62333952）；
  - MBR FAT32 分区识别成功（type=0x0C，start LBA=8192，sectors=62325760）；
  - `fx_media_open` 成功，FAT32、32 扇区/簇；
  - 写入 2KB `DEMO.TXT` + 读回逐字节校验 PASSED；
  - 目录列表显示 `DEMO.TXT`，剩余空间 30416 MB；
  - 写入并 flush 后复位 MCU，二次启动显示 `DEMO.TXT exists, verify PASSED (persisted on card)`，确认数据已真正落盘；
  - 多轮复位无卡死。

### 边界风险

- SD 时钟 30MHz（ClockDiv=4）：CubeMX 重新生成代码会重置 ClockDiv、硬件流控、GPIO 上拉，需手动恢复；
- 若卡是 MBR 但分区不是 FAT（NTFS/exFAT/空分区），demo 会走整卡格式化路径，格式化中途断电可能损坏卡；
- IT 模式依赖 SDMMC1 中断（优先级 0）与 ThreadX 信号量，超时 30s；若中断被禁用或优先级被改，读写会超时失败；
- `fx_media_flush` 仅在存在脏扇区时写卡；拔卡前建议先断电或等待刷盘周期完成；
- newlib-nano 的 printf 不支持 `%llu`，打印 64 位数值需分 32 位段或按 MB 输出；
- SD 卡异常时 demo 打印错误并进入空闲循环，飞控其余任务继续运行，但 SD 功能不可用；
- 待确认：拔卡后在电脑上查看 `DEMO.TXT` 是否可见（板端持久化测试已通过，等待用户 PC 侧确认）。
