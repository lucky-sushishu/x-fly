

#ifndef OSAPI_GLOBALDEFS_H
#define OSAPI_GLOBALDEFS_H

#include "osconfig.h"
#include "osapi-macros.h"

#include "tx_api.h"  /* threadx os file */


typedef TX_THREAD                   OS_THREAD;
typedef TX_SEMAPHORE                OS_SEMAPHORE;
typedef TX_MUTEX                    OS_MUTEX;
typedef TX_QUEUE                    OS_QUEUE;
typedef TX_EVENT_FLAGS_GROUP        OS_EVENT_FLAGS_GROUP;
typedef TX_TIMER                    OS_TIMER;
typedef TX_BLOCK_POOL               OS_BLOCK_POOL;
typedef TX_BYTE_POOL                OS_BYTE_POOL;


#endif /* OSAPI_GLOBALDEFS_H */
