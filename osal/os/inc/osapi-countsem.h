

#ifndef OSAPI_COUNTSEM_H
#define OSAPI_COUNTSEM_H

#include "osapi-globaldefs.h"
#include "tx_api.h" /* threadx os file */

UINT OS_CountSemCreate(OS_SEMAPHORE *semaphore_ptr, CHAR *name_ptr, ULONG initial_count);
UINT OS_CountSemDelete(OS_SEMAPHORE *semaphore_ptr);
UINT OS_CountSemGet(OS_SEMAPHORE *semaphore_ptr, ULONG wait_option);
UINT OS_CountSemGetInfo(OS_SEMAPHORE *semaphore_ptr, CHAR **name, ULONG *current_value, 
                            OS_THREAD **first_suspended, ULONG *suspended_count, OS_SEMAPHORE **next_semaphore);
UINT OS_CountSemPut(OS_SEMAPHORE *semaphore_ptr);

#endif /* OSAPI_COUNTSEM_H */
