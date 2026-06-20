

#ifndef OSAPI_CLOCK_H
#define OSAPI_CLOCK_H

#include "osapi-globaldefs.h"
#include "tx_api.h" /* threadx os file */

ULONG OS_GetLocalTime(VOID);
VOID OS_SetLocalTime(ULONG new_time);

#endif /* OSAPI_CLOCK_H */
