//#include "delay.h"

//void delay_us(uint32_t us)
//{
//	uint32_t ticks;
//	uint32_t told, tnow, tcnt = 0;
//	uint32_t reload;
//	
//	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
//	reload = SysTick->LOAD;
//	ticks = us * 100;
//	told = SysTick->VAL;
//	
//	while(1)
//	{
//		tnow = SysTick->VAL;
//		if(tnow != told)
//		{
//			tnow = SysTick->VAL;
//			if(tnow != told)
//			{
//				if(tnow < told) tcnt += told - tnow;
//				else tcnt += reload - tnow + told;
//				
//				told = tnow;
//				if(tcnt > ticks) break;
//			}
//		}
//	};
//}

//void delay_ms(uint16_t ms)
//{
//	uint32_t i;
//	for(i=0;i<ms;i++) delay_us(1000);
//}

