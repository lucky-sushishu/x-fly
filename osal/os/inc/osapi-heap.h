

#ifndef OSAPI_HEAP_H
#define OSAPI_HEAP_H

#include "osapi-globaldefs.h"
#include "tx_api.h" /* threadx os file */

#define OS_BYTE_POOL_ID             ((ULONG) 0x42595445)

#define OS_UORB_BYTE_POOL_SIZE      (60 * 1024)
extern UCHAR            os_uorb_bytepool_buffer[OS_UORB_BYTE_POOL_SIZE];
extern OS_BYTE_POOL     os_uorb_bytepool;

#define OS_UORB_MAX_NUMS            300
#define OS_UORB_BLOCK_POOL_SIZE     (OS_UORB_MAX_NUMS * (sizeof(OS_MUTEX) + 4))
extern UCHAR            os_uorb_blockpool_buffer[OS_UORB_BLOCK_POOL_SIZE];
extern OS_BLOCK_POOL    os_uorb_blockpool;

/* block pool */
UINT OS_BlockPoolCreate(OS_BLOCK_POOL *pool_ptr, CHAR *name_ptr, UINT block_size,
                        VOID *pool_start, ULONG pool_size);
UINT OS_BlockPoolDelete(OS_BLOCK_POOL *pool_ptr);
UINT OS_BlockPoolInfoGet(OS_BLOCK_POOL *pool_ptr, CHAR **name, ULONG *available, ULONG *total_blocks,
                            TX_THREAD **first_suspended, ULONG *suspended_count, TX_BLOCK_POOL **next_pool);
UINT OS_BlockAllocate(OS_BLOCK_POOL *pool_ptr, VOID **block_ptr, ULONG wait_option);
UINT OS_BlockRelease(VOID *block_ptr);
/* byte pool */
UINT OS_BytePoolCreate(OS_BYTE_POOL *pool_ptr, CHAR *name_ptr, VOID *pool_start, ULONG pool_size);
UINT OS_BytePoolDelete(OS_BYTE_POOL *pool_ptr);
UINT OS_BytePoolInfoGet(OS_BYTE_POOL *pool_ptr, CHAR **name, ULONG *available_bytes, ULONG *fragments,
                        TX_THREAD **first_suspended, ULONG *suspended_count, TX_BYTE_POOL **next_pool);
UINT OS_ByteAllocate(OS_BYTE_POOL *pool_ptr, VOID **memory_ptr, ULONG size, ULONG wait_option);
UINT OS_ByteRelease(VOID *memory_ptr);

#endif /* OSAPI_HEAP_H */
