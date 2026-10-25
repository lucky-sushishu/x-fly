

#include "osapi-mutex.h"

UINT OS_MutSemCreate(OS_MUTEX *mutex_ptr, CHAR *name_ptr, UINT inherit)
{
    return tx_mutex_create(mutex_ptr, name_ptr, inherit);
}

OS_MUTEX *OS_MutSemCreateRet(CHAR *name_ptr, UINT inherit)
{
    OS_MUTEX *mutex_ptr = (OS_MUTEX *)malloc(sizeof(OS_MUTEX));
    OS_MutSemCreate(mutex_ptr, name_ptr, inherit);
    if (mutex_ptr == NULL)
    {
        return NULL;
    }
    return mutex_ptr;
}

UINT OS_MutSemDelete(OS_MUTEX *mutex_ptr)
{
    return tx_mutex_delete(mutex_ptr);
}

UINT OS_MutSemGet(OS_MUTEX *mutex_ptr, ULONG wait_option)
{
    return tx_mutex_get(mutex_ptr, wait_option);
}

UINT OS_MutSemGetInfo(OS_MUTEX *mutex_ptr, CHAR **name, ULONG *count, OS_THREAD **owner, 
                        OS_THREAD **first_suspended, ULONG *suspended_count, OS_MUTEX **next_mutex)
{
    return tx_mutex_info_get(mutex_ptr, name, count, owner, first_suspended, suspended_count, next_mutex);
}

UINT OS_MutSemPut(OS_MUTEX *mutex_ptr)
{
    return tx_mutex_put(mutex_ptr);
}
