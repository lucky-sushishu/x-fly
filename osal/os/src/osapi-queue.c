

#include "osapi-queue.h"

UINT OS_QueueCreate(OS_QUEUE *queue_ptr, CHAR *name_ptr, UINT message_size, VOID *queue_start, ULONG queue_size)
{
    return tx_queue_create(queue_ptr, name_ptr, message_size, queue_start, queue_size);
}

UINT OS_QueueDelete(OS_QUEUE *queue_ptr)
{
    return tx_queue_delete(queue_ptr);
}

UINT OS_QueueInfoGet(OS_QUEUE *queue_ptr, CHAR **name, ULONG *enqueued, ULONG *available_storage,
                            OS_THREAD **first_suspended, ULONG *suspended_count, OS_QUEUE **next_queue)
{
    return tx_queue_info_get(queue_ptr, name, enqueued, available_storage, first_suspended, suspended_count, next_queue);
}

UINT OS_QueueReceive(OS_QUEUE *queue_ptr, VOID *destination_ptr, ULONG wait_option)
{
    return tx_queue_receive(queue_ptr, destination_ptr, wait_option);
}

UINT OS_QueueSend(OS_QUEUE *queue_ptr, VOID *source_ptr, ULONG wait_option)
{
    return tx_queue_send(queue_ptr, source_ptr, wait_option);
}
