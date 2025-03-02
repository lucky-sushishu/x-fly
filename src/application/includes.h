#ifndef __INCLUDES_H
#define __INCLUDES_H

/* rtos文件 */
#include "tx_api.h"

/* 板级驱动 */
#include "gpio.h"
#include "usart.h"
#include "i2c.h"
#include "dma.h"
#include "usb_device.h"

/* BSP */
#include "bsp_dwt.h"
#include "bsp_usb.h"

/* sys文件 */
#include "software_i2c.h"

/* 外部驱动 */
#include "mpu9250.h"


/* Define rtos event flags */
#define IMU_INIT_SUCCEED (0x1 << 0)
#define IMU_INIT_ERROR   (0x1 << 1)

/* Conifg */
#define USE_EULER_RAD 0


#endif
