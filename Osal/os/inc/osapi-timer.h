

#ifndef OSAPI_TIMER_H
#define OSAPI_TIMER_H

#include "osapi-globaldefs.h"
#include "tx_api.h" /* threadx os file */

UINT OS_TimerActivate(OS_TIMER *timer_ptr);
UINT OS_TimerCreate(OS_TIMER *timer_ptr, CHAR *name_ptr, VOID (*expiration_function)(ULONG input),
                    ULONG expiration_input, ULONG initial_ticks, ULONG reschedule_ticks, UINT auto_activate);
UINT OS_TimerDeactivate(OS_TIMER *timer_ptr);
UINT OS_TimerDelete(OS_TIMER *timer_ptr);
UINT OS_TimerInfoGet(OS_TIMER *timer_ptr, CHAR **name, UINT *active, ULONG *remaining_ticks, 
                        ULONG *reschedule_ticks, OS_TIMER **next_timer);

#endif /* OSAPI_TIMER_H */
