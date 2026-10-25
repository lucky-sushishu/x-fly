

#include "osapi-timer.h"

UINT OS_TimerActivate(OS_TIMER *timer_ptr)
{
    return tx_timer_activate(timer_ptr);
}

UINT OS_TimerCreate(OS_TIMER *timer_ptr, CHAR *name_ptr, VOID (*expiration_function)(ULONG input),
                    ULONG expiration_input, ULONG initial_ticks, ULONG reschedule_ticks, UINT auto_activate)
{
    return tx_timer_create(timer_ptr, name_ptr, expiration_function, 
                            expiration_input, initial_ticks, reschedule_ticks, auto_activate);
}

UINT OS_TimerDeactivate(OS_TIMER *timer_ptr)
{
    return tx_timer_deactivate(timer_ptr);
}

UINT OS_TimerDelete(OS_TIMER *timer_ptr)
{
    return tx_timer_delete(timer_ptr);
}

UINT OS_TimerInfoGet(OS_TIMER *timer_ptr, CHAR **name, UINT *active, ULONG *remaining_ticks, 
                        ULONG *reschedule_ticks, OS_TIMER **next_timer)
{
    return tx_timer_info_get(timer_ptr, name, active, remaining_ticks, reschedule_ticks, next_timer);
}
