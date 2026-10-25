

#include "osapi-heap.h"


UCHAR           os_uorb_bytepool_buffer[OS_UORB_BYTE_POOL_SIZE];
OS_BYTE_POOL    os_uorb_bytepool;
UCHAR           os_uorb_blockpool_buffer[OS_UORB_BLOCK_POOL_SIZE];
OS_BLOCK_POOL   os_uorb_blockpool;

/* block pool */
UINT OS_BlockPoolCreate(OS_BLOCK_POOL *pool_ptr, CHAR *name_ptr, UINT block_size,
                        VOID *pool_start, ULONG pool_size)
{
    return(tx_block_pool_create(pool_ptr, name_ptr, block_size, pool_start, pool_size));
}

UINT OS_BlockPoolDelete(OS_BLOCK_POOL *pool_ptr)
{
    return(tx_block_pool_delete(pool_ptr));
}

UINT OS_BlockPoolInfoGet(OS_BLOCK_POOL *pool_ptr, CHAR **name, ULONG *available, ULONG *total_blocks,
                            TX_THREAD **first_suspended, ULONG *suspended_count, TX_BLOCK_POOL **next_pool)
{
    return(tx_block_pool_info_get(pool_ptr, name, available, total_blocks, first_suspended,
           suspended_count, next_pool));
}

UINT OS_BlockAllocate(OS_BLOCK_POOL *pool_ptr, VOID **block_ptr, ULONG wait_option)
{
    return(tx_block_allocate(pool_ptr, block_ptr, wait_option));
}

UINT OS_BlockRelease(VOID *block_ptr)
{
    return(tx_block_release(block_ptr));
}

/* byte pool */
UINT OS_BytePoolCreate(OS_BYTE_POOL *pool_ptr, CHAR *name_ptr, VOID *pool_start, ULONG pool_size)
{
    return(tx_byte_pool_create(pool_ptr, name_ptr, pool_start, pool_size));
}

UINT OS_BytePoolDelete(OS_BYTE_POOL *pool_ptr)
{
    return(tx_byte_pool_delete(pool_ptr));
}

UINT OS_BytePoolInfoGet(OS_BYTE_POOL *pool_ptr, CHAR **name, ULONG *available_bytes, ULONG *fragments,
                        TX_THREAD **first_suspended, ULONG *suspended_count, TX_BYTE_POOL **next_pool)
{
    return(tx_byte_pool_info_get(pool_ptr, name, available_bytes, fragments,
           first_suspended, suspended_count, next_pool));
}

UINT OS_ByteAllocate(OS_BYTE_POOL *pool_ptr, VOID **memory_ptr, ULONG size, ULONG wait_option)
{
    return(tx_byte_allocate(pool_ptr, memory_ptr, size, wait_option));
}

UINT OS_ByteRelease(VOID *memory_ptr)
{
    return(tx_byte_release(memory_ptr));
}
