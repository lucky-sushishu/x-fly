

#include "osapi-event_flags.h"

UINT OS_EventFlagsCreate(OS_EVENT_FLAGS_GROUP *group_ptr, CHAR *name_ptr)
{
    return tx_event_flags_create(group_ptr, name_ptr);
}

UINT OS_EventFlagsDelete(OS_EVENT_FLAGS_GROUP *group_ptr)
{
    return tx_event_flags_delete(group_ptr);
}

UINT OS_EventFlagsGet(OS_EVENT_FLAGS_GROUP *group_ptr, ULONG requested_flags,
                        UINT get_option, ULONG *actual_flags_ptr, ULONG wait_option)
{
    return tx_event_flags_get(group_ptr, requested_flags, get_option, actual_flags_ptr, wait_option);
}

UINT OS_EventFlagsInfoGet(OS_EVENT_FLAGS_GROUP *group_ptr, CHAR **name, ULONG *current_flags, 
                            OS_THREAD **first_suspended, ULONG *suspended_count, OS_EVENT_FLAGS_GROUP **next_group)
{
    return tx_event_flags_info_get(group_ptr, name, current_flags, first_suspended, suspended_count, next_group);
}

UINT OS_EventFlagsSet(OS_EVENT_FLAGS_GROUP *group_ptr, ULONG flags_to_set, UINT set_option)
{
    return tx_event_flags_set(group_ptr, flags_to_set, set_option);
}

UINT OS_EventFlagsSetNotify(OS_EVENT_FLAGS_GROUP *group_ptr, VOID (*events_set_notify)(OS_EVENT_FLAGS_GROUP *notify_group_ptr))
{
    return tx_event_flags_set_notify(group_ptr, events_set_notify);
}
