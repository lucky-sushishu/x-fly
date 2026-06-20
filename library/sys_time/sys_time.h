#ifndef SYS_TIME_H
#define SYS_TIME_H

#include <stdint.h>

#define SYS_TICKS_LOAD (480000UL)


uint32_t sys_tick_get(void);
uint64_t sys_absolute_time(void);
uint64_t sys_elapsed_time(uint64_t t);
uint32_t sys_absolute_time_ms(void);
uint32_t sys_elapsed_time_ms(uint32_t t);

#endif
