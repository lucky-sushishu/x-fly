#include "sys_time.h"
#include "stm32h743xx.h"
#include "osapi.h"

uint32_t sys_tick_get(void)
{
	return OS_GetLocalTime();
}


uint64_t sys_absolute_time(void)
{
	return (uint64_t)((uint64_t)(sys_tick_get())*1000UL + ((SYS_TICKS_LOAD - SysTick->VAL - 1) * 1000UL)/SYS_TICKS_LOAD);
}


uint64_t sys_elapsed_time(uint64_t t)
{
	return (uint64_t)(((uint64_t)(sys_tick_get())*1000UL + ((SYS_TICKS_LOAD - SysTick->VAL - 1) * 1000UL)/SYS_TICKS_LOAD) -t);
}



uint32_t sys_absolute_time_ms(void)
{
	return sys_tick_get();
}


uint32_t sys_elapsed_time_ms(uint32_t t)
{
	return (sys_tick_get() - t);
}
