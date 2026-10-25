

#ifndef OSAL_EVENT_FLAGS_H
#define OSAL_EVENT_FLAGS_H

#include "osapi-globaldefs.h"
#include "tx_api.h" /* threadx os file */

UINT OS_EventFlagsCreate(OS_EVENT_FLAGS_GROUP *group_ptr, CHAR *name_ptr);
UINT OS_EventFlagsDelete(OS_EVENT_FLAGS_GROUP *group_ptr);
UINT OS_EventFlagsGet(OS_EVENT_FLAGS_GROUP *group_ptr, ULONG requested_flags,
                        UINT get_option, ULONG *actual_flags_ptr, ULONG wait_option);
UINT OS_EventFlagsInfoGet(OS_EVENT_FLAGS_GROUP *group_ptr, CHAR **name, ULONG *current_flags, 
                            OS_THREAD **first_suspended, ULONG *suspended_count, OS_EVENT_FLAGS_GROUP **next_group);
UINT OS_EventFlagsSet(OS_EVENT_FLAGS_GROUP *group_ptr, ULONG flags_to_set, UINT set_option);
UINT OS_EventFlagsSetNotify(OS_EVENT_FLAGS_GROUP *group_ptr, VOID (*events_set_notify)(TX_EVENT_FLAGS_GROUP *notify_group_ptr));

#endif /* OSAL_EVENT_FLAGS_H */
