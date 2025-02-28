#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "stm32f4xx_hal.h"

#define  DWT_CYCCNT  *((volatile unsigned int *)0xE0001004)
#define  DWT_CTRL    *((volatile unsigned int *)0xE0001000)
#define  DEMCR       *((volatile unsigned int *)0xE000EDFC)
	
#define  DEM_CR_TRCENA               (1 << 24)
#define  DWT_CR_CYCCNTENA            (1 <<  0)

void BSP_DWT_Init(void);

#endif
