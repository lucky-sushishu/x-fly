

#include "osapi-countsem.h"

UINT OS_CountSemCreate(OS_SEMAPHORE *semaphore_ptr, CHAR *name_ptr, ULONG initial_count)
{
    return tx_semaphore_create(semaphore_ptr, name_ptr, initial_count);
}

UINT OS_CountSemDelete(OS_SEMAPHORE *semaphore_ptr)
{
    return tx_semaphore_delete(semaphore_ptr);
}

UINT OS_CountSemGet(OS_SEMAPHORE *semaphore_ptr, ULONG wait_option)
{
    return tx_semaphore_get(semaphore_ptr, wait_option);
}

UINT OS_CountSemGetInfo(OS_SEMAPHORE *semaphore_ptr, CHAR **name, ULONG *current_value, 
                            OS_THREAD **first_suspended, ULONG *suspended_count, OS_SEMAPHORE **next_semaphore)
{
    return tx_semaphore_info_get(semaphore_ptr, name, current_value,
                                    first_suspended, suspended_count, next_semaphore);
}

UINT OS_CountSemPut(OS_SEMAPHORE *semaphore_ptr)
{
    return tx_semaphore_put(semaphore_ptr);
}
