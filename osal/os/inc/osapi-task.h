

#ifndef OSAPI_TASK_H
#define OSAPI_TASK_H

#include "osapi-globaldefs.h"
#include "tx_api.h" /* threadx os file */

/** @defgroup OSAPITask OSAL Task APIs
 * @{
 */

UINT OS_TaskCreate(OS_THREAD *thread_ptr, CHAR *name_ptr, VOID (*entry_function)(ULONG id), ULONG entry_input,
                        VOID *stack_start, ULONG stack_size, UINT priority, UINT preempt_threshold, ULONG time_slice, UINT auto_start);
UINT OS_TaskDelete(OS_THREAD *thread_ptr);
UINT OS_TaskDelay(ULONG ticks);
UINT OS_TaskSuspend(OS_THREAD *thread_ptr);
UINT OS_TaskResume(OS_THREAD *thread_ptr);
UINT OS_TaskTerminate(OS_THREAD *thread_ptr);
UINT OS_TaskReset(OS_THREAD *thread_ptr);
UINT OS_TaskSetPriority(OS_THREAD *thread_ptr, UINT new_priority, UINT *old_priority);
OS_THREAD *OS_TaskGetCurrent(VOID);
UINT OS_TaskGetInfo(OS_THREAD *thread_ptr, CHAR **name, UINT *state, ULONG *run_count, UINT *priority,
                        UINT *preemption_threshold, ULONG *time_slice, OS_THREAD **next_thread, OS_THREAD **next_suspended_thread);
UINT OS_TaskEntryExitNotify(OS_THREAD *thread_ptr, VOID (*thread_entry_exit_notify)(TX_THREAD *notify_thread_ptr, UINT type));

/**@}*/

#endif /* OSAPI_TASK_H */
