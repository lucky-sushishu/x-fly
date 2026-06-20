

#ifndef OSAPI_QUEUE_H
#define OSAPI_QUEUE_H

#include "osapi-globaldefs.h"
#include "tx_api.h" /* threadx os file */

UINT OS_QueueCreate(OS_QUEUE *queue_ptr, CHAR *name_ptr, UINT message_size, VOID *queue_start, ULONG queue_size);
UINT OS_QueueDelete(OS_QUEUE *queue_ptr);
UINT OS_QueueInfoGet(OS_QUEUE *queue_ptr, CHAR **name, ULONG *enqueued, ULONG *available_storage,
                            OS_THREAD **first_suspended, ULONG *suspended_count, OS_QUEUE **next_queue);
UINT OS_QueueReceive(OS_QUEUE *queue_ptr, VOID *destination_ptr, ULONG wait_option);
UINT OS_QueueSend(OS_QUEUE *queue_ptr, VOID *source_ptr, ULONG wait_option);

#endif /* OSAPI_QUEUE_H */
