

#ifndef OSAPI_UORB_H
#define OSAPI_UORB_H

#include "osapi-globaldefs.h"
#include "tx_api.h" /* threadx os file */
#include "osapi-mutex.h"
#include <stdbool.h>
#include <stdint.h>

#define UORB_MUTEX_OK    (0)
#define UORB_MUTEX_ERROR (-1)

#define UORB_OK     (1)
#define UORB_ERROR  (-1)

#define UORB_MAX_QUEUE_SIZE (10) /* 最大缓存的数量 */
#define UORB_MAX_MULTI_SIZE (10) /* 最大实例的数量 */

typedef struct uorb_sub_s {
    uint64_t time;
    uint32_t instance;
    uint32_t generation;
} uorb_sub_t;

typedef OS_MUTEX *uorb_mutex_id;

/* Structure Definition */
typedef struct uorb_node_s {
    bool                is_advertised;
    uint64_t            timestamp;
    uint32_t            instance;
    uorb_mutex_id       mtx_lock;
    void               *data;
    uint32_t            queue_size;
    uint32_t            generation;
    struct uorb_node_s *next;
} uorb_node_t;

struct uorb_metadata {
    const CHAR      *o_name;
    const UINT       o_size;
    uint32_t         o_number_of_instances;
    uorb_node_t     *node;
};

typedef struct uorb_sub_struct_s {
    bool                    update;
    struct uorb_metadata   *uorb_id;
    uorb_sub_t              uorb_sub;
    uint8_t                *uorb_data_addr;
} uorb_sub_struct;

/**
 * uorb timestamp_get call-back function typedef
 * @param  void NULL
 * @return      timestamp in us
 */
typedef uint64_t (*uorb_timestamp_get)(void);

/**
 * SORB Mutex Creat call-back function typedef
 * @param  void * mutex creat input object pointer
 * @return SORB Mutex ID
 */
typedef int (*uorb_mutex_create)(struct uorb_node_s *node);

/**
 * SORB Mutex Wait  call-back function typedef
 * @param  mutex_id SORB Mutex ID
 * @param  millisec wait time in millisec 
 * @return          SORB Mutex Status UORB_MUTEX_OK or UORB_MUTEX_ERROR
 */
typedef int (*uorb_mutex_take)(struct uorb_node_s *node, UINT millisec);

/**
 * SORB Mutex Release call-back function typedef
 * @param  mutex_id SORB Mutex ID
 * @return          SORB Mutex Status UORB_MUTEX_OK or UORB_MUTEX_ERROR
 */
typedef int (*uorb_mutex_release)(struct uorb_node_s *node);

/**
 * SORB malloc function typedef
 * @param  size     malloc's size
 * @return          
 */
typedef uint32_t (*uorb_malloc)(VOID **memory_ptr, uint32_t size);

/**
 * SORB delay call-back function
 * @param  millisec delay time in millisec
 * @return          osDelay status 0 is OK
 */
typedef void (*uorb_delay)(UINT millisec);


/** Macro Definition
-----------------------------------------------------------------------------*/
#define UORB_ID(_name) &__uorb_##_name
#define UORB_GROUP_ID(_group, _id, _number) &__uorb_##_group##_##_id##_##_number

#if defined(__cplusplus)
#define UORB_DECLARE(_name)     \
    extern "C" struct uorb_metadata __uorb_##_name // __EXPORT
#define UORB_DECLEAR_OPTIONAL(_name)    \
    extern "C" struct uorb_metadata __uorb_##_name __EXPORT __attribute__((weak))
#else
#define UORB_DECLARE(_name)     \
    extern struct uorb_metadata __uorb_##_name // __EXPORT
#define UORB_DECLEAR_OPTIONAL(_name)    \
    extern struct uorb_metadata __uorb_##_name __EXPORT __attribute__((weak))
#endif

#define UORB_BUFF_HEAD(_name)       (__uorb_##name.buff)

#define UORB_DEFINE(_name, _struct)                         \
                struct uorb_metadata __uorb_##_name = {     \
                        #_name,                             \
                        sizeof(_struct),                    \
                        0,                                  \
                        NULL                                \
                }

#ifdef __cplusplus
extern "C"
{
#endif

/** Function Declaration
-----------------------------------------------------------------------------*/
/**
 * SORB Initialize Function
 * @param get_timestamp      sorb timestamp get call-back function
 * @param uorb_mutex_create  sorb mutex creat call-back function
 * @param uorb_mutex_take    sorb mutex wait call-back function
 * @param uorb_mutex_release sorb mutex release call-back function
 * @param uorb_delay         sorb delay call-back function
 * @param uorb_malloc        sorb malloc
 */
void uorb_initialize(uorb_timestamp_get get_timestamp,
                     uorb_mutex_create  uorb_mutex_create,
                     uorb_mutex_take    uorb_mutex_take,
                     uorb_mutex_release uorb_mutex_release,
                     uorb_delay         uorb_delay,
                     uorb_malloc        uorb_malloc);

/**
 * SORB advertise function
 * @param  meta uorb metadata struct pointer
 * @return      UORB_OK or SORB_SRROR
 */
int uorb_advertise(struct uorb_metadata *meta);

int uorb_advertise_multi(struct uorb_metadata *meta, int instance);
int uorb_advertise_queue(struct uorb_metadata *meta, uint32_t queue_size);
int uorb_advertise_multi_queue(struct uorb_metadata *meta, int instance, uint32_t queue_size);

uorb_node_t *uorb_node_find(struct uorb_metadata *meta, uint32_t instance);
uorb_node_t *uorb_node_creat(struct uorb_metadata *meta, uint32_t instance);

/**
 * SORB data publish function
 * @param  meta metadata struct pointer
 * @param  data publish data pointer
 * @return      UORB_OK or UORB_ERROR
 */
int uorb_publish(struct uorb_metadata *meta, const void *data);

int uorb_publish_multi(struct uorb_metadata *meta, const void *data, uint32_t instance);

/**
 * SORB subscrible
 * @param  meta uorb metadata struct pointer
 * @return      subscrible time handle 0
 */
uorb_sub_t uorb_subscribe(struct uorb_metadata *meta);

uorb_sub_t uorb_subscribe_multi(struct uorb_metadata *meta, int instance);

/**
 * SORB check 
 * @param  meta     uorb metadata struct pointer
 * @param  time_sub uorb subscrible time-handle
 * @param  update   update or not
 * @return          UORB_OK or UORB_ERROR
 */
int uorb_check(struct uorb_metadata *meta, uorb_sub_t time_sub, bool *update);

/**
 * SORB meta data copy function
 * @param  meta     metadata struct pointer
 * @param  time_sub subscrible time handle
 * @param  buff     copy target buffer
 * @return          UORB_OK or UORB_ERROR
 */
int uorb_copy(struct uorb_metadata *meta, uorb_sub_t *time_sub, void *buff);

/**
 * SORB refresh function
 * @param  meta     metadata struct pointer
 * @param  time_sub subscrible time handle
 * @return          UORB_OK or UORB_ERROR
 */
int uorb_refresh(struct uorb_metadata *meta, uorb_sub_t *time_sub);

// /**
//  * copy uorb metadata to buffer if meta-data is updated
//  * @param  meta     metadata struct pointer
//  * @param  time_sub subscrible time-handle
//  * @param  buff     copy target buffer
//  * @return          UORB_OK or UORB_ERROR
//  */
// int copy_if_update(struct uorb_metadata *meta, uorb_sub_t *time_sub, void *buff);

#endif /* OSAPI_UORB_H */
