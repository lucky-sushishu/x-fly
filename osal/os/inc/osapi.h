
#ifndef OSAPI_H
#define OSAPI_H

#include "osapi-version.h"

/*
** Include the configuration file
*/
#include "osconfig.h"

#define OS_MAX_PRIORITIES               TX_MAX_PRIORITIES
#define OS_MINIMUM_STACK                TX_MINIMUM_STACK

/*
** Include the OS API modules
*/
#include "osapi-task.h"                 /* task API */
#include "osapi-clock.h"                /* clock API */
#include "osapi-countsem.h"             /* counting semaphore API */
#include "osapi-mutex.h"                /* mutex API */
#include "osapi-queue.h"                /* queue API */
#include "osapi-event_flags.h"          /* event flags API */
#include "osapi-timebase.h"             /* time API */
#include "osapi-timer.h"                /* time API */
#include "osapi-uorb.h"                 /* uorb API */
#include "osapi-heap.h"                 /* heap API */
#include "osapi-shell.h"                /* shell API */
#include "osapi-filesys.h"              /* filesys API */

#include "tx_port.h"

#define OS_KernelEnter                  tx_kernel_enter
#define OS_AppDefine(f)                 tx_application_define(f)

#define OS_INTERRUPT_SAVE_AREA          TX_INTERRUPT_SAVE_AREA
#define OS_DISABLE                      TX_DISABLE
#define OS_RESTORE                      TX_RESTORE

#endif /* OSAPI_H */
