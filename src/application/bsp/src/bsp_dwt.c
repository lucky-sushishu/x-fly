#include "bsp_dwt.h"

void BSP_DWT_Init(void)
{
        DEMCR          |= (unsigned int)DEM_CR_TRCENA;   /* Enable Cortex-M4's DWT CYCCNT reg.  */
        DWT_CYCCNT      = (unsigned int)0u;
        DWT_CTRL       |= (unsigned int)DWT_CR_CYCCNTENA;
}
