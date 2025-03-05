#ifndef _BSP_USB_H
#define _BSP_USB_H

#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>

void VirtualComPort_Printf(const char *format, ...);
uint8_t VirtualComPort_Sned(uint8_t* Buf, uint16_t Len);

#endif
