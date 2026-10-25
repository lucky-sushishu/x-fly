

#ifndef OSAPI_MUTEX_H
#define OSAPI_MUTEX_H

#include "osapi-globaldefs.h"
#include "tx_api.h" /* threadx os file */

UINT OS_MutSemCreate(OS_MUTEX *mutex_ptr, CHAR *name_ptr, UINT inherit);
OS_MUTEX *OS_MutSemCreateRet(CHAR *name_ptr, UINT inherit);
UINT OS_MutSemDelete(OS_MUTEX *mutex_ptr);
UINT OS_MutSemGet(OS_MUTEX *mutex_ptr, ULONG wait_option);
UINT OS_MutSemGetInfo(OS_MUTEX *mutex_ptr, CHAR **name, ULONG *count, OS_THREAD **owner, 
                        OS_THREAD **first_suspended, ULONG *suspended_count, OS_MUTEX **next_mutex);
UINT OS_MutSemPut(OS_MUTEX *mutex_ptr);

#endif /* OSAPI_MUTEX_H */
