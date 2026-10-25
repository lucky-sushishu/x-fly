

#include "osapi-task.h"

UINT OS_TaskCreate(OS_THREAD *thread_ptr, CHAR *name_ptr, VOID (*entry_function)(ULONG id), ULONG entry_input,
                    VOID *stack_start, ULONG stack_size, UINT priority, UINT preempt_threshold, ULONG time_slice, UINT auto_start)
{
    return tx_thread_create(thread_ptr, name_ptr, entry_function, entry_input,
                             stack_start, stack_size, priority, preempt_threshold, time_slice, auto_start);
}

UINT OS_TaskDelete(OS_THREAD *thread_ptr)
{
    return tx_thread_delete(thread_ptr);
}

UINT OS_TaskDelay(ULONG ticks)
{
    return tx_thread_sleep(ticks);
}

UINT OS_TaskSuspend(OS_THREAD *thread_ptr)
{
    return tx_thread_suspend(thread_ptr);
}

UINT OS_TaskResume(OS_THREAD *thread_ptr)
{
    return tx_thread_resume(thread_ptr);
}

UINT OS_TaskTerminate(OS_THREAD *thread_ptr)
{
    return tx_thread_terminate(thread_ptr);
}

UINT OS_TaskReset(OS_THREAD *thread_ptr)
{
    return tx_thread_reset(thread_ptr);
}

UINT OS_TaskSetPriority(OS_THREAD *thread_ptr, UINT new_priority, UINT *old_priority)
{
    return tx_thread_priority_change(thread_ptr, new_priority, old_priority);
}

OS_THREAD *OS_TaskGetCurrent(VOID)
{
    return tx_thread_identify();
}

UINT OS_TaskGetInfo(OS_THREAD *thread_ptr, CHAR **name, UINT *state, ULONG *run_count, UINT *priority,
                        UINT *preemption_threshold, ULONG *time_slice, OS_THREAD **next_thread, OS_THREAD **next_suspended_thread)
{
    return tx_thread_info_get(thread_ptr, name, state, run_count, priority,
                        preemption_threshold, time_slice, next_thread, next_suspended_thread);
}

UINT OS_TaskEntryExitNotify(OS_THREAD *thread_ptr, VOID (*thread_entry_exit_notify)(TX_THREAD *notify_thread_ptr, UINT type))
{
    return tx_thread_entry_exit_notify(thread_ptr, thread_entry_exit_notify);
}
