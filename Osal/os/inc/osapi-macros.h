

/**
 * \file
 *
 * Macro definitions that are used across all OSAL subsystems
 */

#ifndef OSAPI_MACROS_H
#define OSAPI_MACROS_H

#include "tx_api.h" /* threadx os file */

/* OSAL API input parameters and general constants.  */

#define OS_NO_WAIT                      ((ULONG)  0)
#define OS_WAIT_FOREVER                 ((ULONG)  0xFFFFFFFFUL)
#define OS_AND                          ((UINT)   2)
#define OS_AND_CLEAR                    ((UINT)   3)
#define OS_OR                           ((UINT)   0)
#define OS_OR_CLEAR                     ((UINT)   1)
#define OS_1_ULONG                      ((UINT)   1)
#define OS_2_ULONG                      ((UINT)   2)
#define OS_4_ULONG                      ((UINT)   4)
#define OS_8_ULONG                      ((UINT)   8)
#define OS_16_ULONG                     ((UINT)   16)
#define OS_NO_TIME_SLICE                ((ULONG)  0)
#define OS_AUTO_START                   ((UINT)   1)
#define OS_DONT_START                   ((UINT)   0)
#define OS_AUTO_ACTIVATE                ((UINT)   1)
#define OS_NO_ACTIVATE                  ((UINT)   0)
#define OS_TRUE                         ((UINT)   1)
#define OS_FALSE                        ((UINT)   0)
#define OS_NULL                         ((void *) 0)
#define OS_INHERIT                      ((UINT)   1)
#define OS_NO_INHERIT                   ((UINT)   0)
#define OS_THREAD_ENTRY                 ((UINT)   0)
#define OS_THREAD_EXIT                  ((UINT)   1)
#define OS_NO_SUSPENSIONS               ((UINT)   0)
#define OS_NO_MESSAGES                  ((UINT)   0)
#define OS_EMPTY                        ((ULONG)  0)
#define OS_CLEAR_ID                     ((ULONG)  0)
#define OS_STACK_FILL                   ((ULONG)  0xEFEFEFEFUL)


/* OS execution state values.  */

#define OS_READY                        ((UINT) 0)
#define OS_COMPLETED                    ((UINT) 1)
#define OS_TERMINATED                   ((UINT) 2)
#define OS_SUSPENDED                    ((UINT) 3)
#define OS_SLEEP                        ((UINT) 4)
#define OS_QUEUE_SUSP                   ((UINT) 5)
#define OS_SEMAPHORE_SUSP               ((UINT) 6)
#define OS_EVENT_FLAG                   ((UINT) 7)
#define OS_BLOCK_MEMORY                 ((UINT) 8)
#define OS_BYTE_MEMORY                  ((UINT) 9)
#define OS_IO_DRIVER                    ((UINT) 10)
#define OS_FILE                         ((UINT) 11)
#define OS_TCP_IP                       ((UINT) 12)
#define OS_MUTEX_SUSP                   ((UINT) 13)
#define OS_PRIORITY_CHANGE              ((UINT) 14)


/* OSAL API return values.  */

#define OS_SUCCESS                      ((UINT) 0x00)
#define OS_DELETED                      ((UINT) 0x01)
#define OS_NO_MEMORY                    ((UINT) 0x10)
#define OS_POOL_ERROR                   ((UINT) 0x02)
#define OS_PTR_ERROR                    ((UINT) 0x03)
#define OS_WAIT_ERROR                   ((UINT) 0x04)
#define OS_SIZE_ERROR                   ((UINT) 0x05)
#define OS_GROUP_ERROR                  ((UINT) 0x06)
#define OS_NO_EVENTS                    ((UINT) 0x07)
#define OS_OPTION_ERROR                 ((UINT) 0x08)
#define OS_QUEUE_ERROR                  ((UINT) 0x09)
#define OS_QUEUE_EMPTY                  ((UINT) 0x0A)
#define OS_QUEUE_FULL                   ((UINT) 0x0B)
#define OS_SEMAPHORE_ERROR              ((UINT) 0x0C)
#define OS_NO_INSTANCE                  ((UINT) 0x0D)
#define OS_THREAD_ERROR                 ((UINT) 0x0E)
#define OS_PRIORITY_ERROR               ((UINT) 0x0F)
#define OS_START_ERROR                  ((UINT) 0x10)
#define OS_DELETE_ERROR                 ((UINT) 0x11)
#define OS_RESUME_ERROR                 ((UINT) 0x12)
#define OS_CALLER_ERROR                 ((UINT) 0x13)
#define OS_SUSPEND_ERROR                ((UINT) 0x14)
#define OS_TIMER_ERROR                  ((UINT) 0x15)
#define OS_TICK_ERROR                   ((UINT) 0x16)
#define OS_ACTIVATE_ERROR               ((UINT) 0x17)
#define OS_THRESH_ERROR                 ((UINT) 0x18)
#define OS_SUSPEND_LIFTED               ((UINT) 0x19)
#define OS_WAIT_ABORTED                 ((UINT) 0x1A)
#define OS_WAIT_ABORT_ERROR             ((UINT) 0x1B)
#define OS_MUTEX_ERROR                  ((UINT) 0x1C)
#define OS_NOT_AVAILABLE                ((UINT) 0x1D)
#define OS_NOT_OWNED                    ((UINT) 0x1E)
#define OS_INHERIT_ERROR                ((UINT) 0x1F)
#define OS_NOT_DONE                     ((UINT) 0x20)
#define OS_CEILING_EXCEEDED             ((UINT) 0x21)
#define OS_INVALID_CEILING              ((UINT) 0x22)
#define OS_FEATURE_NOT_ENABLED          ((UINT) 0xFF)

#endif /* OSAPI_MACROS_H */
