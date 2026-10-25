

#include "osapi-clock.h"

ULONG OS_GetLocalTime(VOID)
{
    return tx_time_get();
}

VOID OS_SetLocalTime(ULONG new_time)
{
    tx_time_set(new_time);
}
