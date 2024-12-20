#ifndef __INCLUDES_H
#define __INCLUDES_H

/* rtos文件 */
#include "tx_api.h"

/* 板级驱动 */
#include "gpio.h"
#include "usart.h"

/* sys文件 */
#include "i2c.h"

/* 外部驱动 */
#include "mpu9250.h"


/* Define rtos event flags */
#define IMU_INIT_SUCCEED (0x1 << 0)
#define IMU_INIT_ERROR   (0x1 << 1)

#endif
