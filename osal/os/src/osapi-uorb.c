

#include "osapi-uorb.h"
#include <string.h>

/* Macro Definition ---------------------------------------------------------*/
#define    UORB_LOCK_READING   0x01
#define    UORB_LOCK_WRITING   0x02
#define    UORB_LOCK_RELEASE   0x00

#define    UORB_INIT_TIMESTAMP   (0)


/* Global Variables ---------------------------------------------------------*/

static uorb_timestamp_get _uorb_timestamp_get;
static uorb_mutex_create  _uorb_mutex_create;
static uorb_mutex_take    _uorb_mutex_take;
static uorb_mutex_release _uorb_mutex_release;
// static uorb_delay         _uorb_delay;
static uorb_malloc        _uorb_malloc;


/* Function Implantation ----------------------------------------------------*/
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
                     uorb_malloc        uorb_malloc)
{
    _uorb_timestamp_get = get_timestamp;
    _uorb_mutex_create  = uorb_mutex_create;
    _uorb_mutex_take    = uorb_mutex_take;
    _uorb_mutex_release = uorb_mutex_release;
    // _uorb_delay            = uorb_delay;
    _uorb_malloc        = uorb_malloc;

}

/**
 * SORB advertise function
 * @param  meta sorb metadata struct pointer
 * @return      UORB_OK or SORB_SRROR
 */
int uorb_advertise(struct uorb_metadata *meta)
{
    return uorb_advertise_multi_queue(meta, 0, 0);
}

int uorb_advertise_multi(struct uorb_metadata *meta, int instance)
{
    return uorb_advertise_multi_queue(meta, instance, 0);
}

int uorb_advertise_queue(struct uorb_metadata *meta, uint32_t queue_size)
{
    return uorb_advertise_multi_queue(meta, 0, queue_size);
}

int uorb_advertise_multi_queue(struct uorb_metadata *meta, int instance, uint32_t queue_size)
{
    if (meta == NULL || queue_size > UORB_MAX_QUEUE_SIZE)
    {
        return UORB_ERROR;
    }

    /* if no found instance, creat a new node */
    uorb_node_t *instance_node = uorb_node_find(meta, instance);
    if (instance_node == NULL)
    {
        uorb_node_t *node = uorb_node_creat(meta, instance);

        node->timestamp = UORB_INIT_TIMESTAMP; /// when advertise a sorb, init timestamp MUST BE 0.
                                        /// to prevent random data copy
        if(node->mtx_lock == NULL ) {
            uint32_t res = _uorb_mutex_create(node);
            if (res != 0)
                return UORB_ERROR;
        }

        node->generation    = 0;
        node->queue_size  = (queue_size == 0) ? 1 : queue_size;
        node->is_advertised = true;

        if (_uorb_malloc((void **)&node->data, meta->o_size * node->queue_size) != 0)
        {
            return UORB_ERROR;
        }
    }
    else
    {
        if (instance_node->is_advertised)
        {
            return UORB_ERROR;
        }
    }

    return UORB_OK;
}

uorb_node_t *uorb_node_find(struct uorb_metadata *meta, uint32_t instance)
{
    if (meta == NULL)
    {
        return NULL;
    }

    uorb_node_t *object = meta->node;
    uint32_t max = meta->o_number_of_instances;
    while (object != NULL && max-- != 0)
    {
        if (object->instance == instance) {
            return object;
        }
        object = object->next;
    }
    return NULL;
}

uorb_node_t *uorb_node_creat(struct uorb_metadata *meta, uint32_t instance)
{
    if (meta->o_number_of_instances > UORB_MAX_MULTI_SIZE)
    {
        return NULL;
    }

    uorb_node_t *new = NULL;

    if (_uorb_malloc((void **)&new, sizeof(uorb_node_t)) != 0)
    {
        return NULL;
    }

    new->next          = NULL;
    new->instance      = instance;
    new->is_advertised = false;

    if (meta->node == NULL) {
        meta->node = new;
        meta->o_number_of_instances++;
        return new;
    }

    uorb_node_t *now = meta->node;
    uint32_t max     = meta->o_number_of_instances;
    do
    {
        if (now == NULL) {
            now = new;
        }
        else if (now->next == NULL) {
            now->next = new;
        }
        now = now->next;
    } while (now != NULL && --max != 0);

    meta->o_number_of_instances++;
    return new;
}

/**
 * SORB data publish function
 * @param  meta metadata struct pointer
 * @param  data publish data pointer
 * @return      UORB_OK or UORB_ERROR
 */
int uorb_publish(struct uorb_metadata *meta, const void *data)
{
    return uorb_publish_multi(meta, data, 0);
}

int uorb_publish_multi(struct uorb_metadata *meta, const void *data, uint32_t instance)
{
    uorb_node_t *node = uorb_node_find(meta, instance);
    if (node == NULL) {
        return UORB_ERROR;
    }

    if(UORB_MUTEX_OK == _uorb_mutex_take(node, 1)) {
        node->timestamp = _uorb_timestamp_get();
        memcpy((uint8_t *)node->data + ((node->generation % node->queue_size) * meta->o_size), data, meta->o_size);
        node->generation++;
        _uorb_mutex_release(node);
        return UORB_OK;
    }
    return UORB_ERROR;
}

/**
 * SORB subscrible
 * @param  meta sorb metadata struct pointer
 * @return      subscrible timestamp  �� current timestamp
 */
uorb_sub_t uorb_subscribe(struct uorb_metadata *meta)
{
    return uorb_subscribe_multi(meta, 0);
}

uorb_sub_t uorb_subscribe_multi(struct uorb_metadata *meta, int instance)
{
    uorb_sub_t sub;
    sub.time       = UORB_INIT_TIMESTAMP;
    sub.instance   = instance;
    sub.generation = 0;
	return sub;
}

/**
 * SORB check 
 * @param  meta     sorb metadata struct pointer
 * @param  time_sub sorb subscrible time-handle
 * @param  update   update or not
 * @return          UORB_OK or UORB_ERROR
 */
int uorb_check(struct uorb_metadata *meta, uorb_sub_t time_sub, bool *update)
{
    uorb_node_t *node = uorb_node_find(meta, time_sub.instance);
    if (node == NULL) {
        return UORB_ERROR;
    }

    if (node->timestamp != time_sub.time || node->generation != time_sub.generation)
    {
        *update = true;
    } else
    {
        *update = false;
    }
    return (*update == true) ? UORB_OK : UORB_ERROR;
}

/**
 * SORB meta data copy function
 * @param  meta     metadata struct pointer
 * @param  time_sub subscrible time handle
 * @param  buff     copy target buffer
 * @return          UORB_OK or UORB_ERROR
 */
int uorb_copy(struct uorb_metadata *meta, uorb_sub_t *time_sub, void *buff)
{
    uorb_node_t *node = uorb_node_find(meta, time_sub->instance);
    if (node == NULL) {
        return UORB_ERROR;
    }

    if(node->mtx_lock == OS_NULL) {
        return UORB_ERROR;
    }
    if(UORB_MUTEX_OK == _uorb_mutex_take(node, 1)) {
        (*time_sub).time = node->timestamp;
        memcpy(buff, (uint8_t *)node->data + ((time_sub->generation % node->queue_size) * meta->o_size), meta->o_size);
        time_sub->generation++;
        _uorb_mutex_release(node);
        return UORB_OK;
    }
    return UORB_ERROR;
}

/**
 * SORB refresh function
 * @param  meta     metadata struct pointer
 * @param  time_sub subscrible time handle
 * @return          UORB_OK or UORB_ERROR
 */
int uorb_refresh(struct uorb_metadata *meta, uorb_sub_t *time_sub)
{
    uorb_node_t *node = uorb_node_find(meta, time_sub->instance);
    if (node == NULL) {
        return UORB_ERROR;
    }

    if(node->mtx_lock == OS_NULL) {
        return UORB_ERROR;
    }
    if(UORB_MUTEX_OK == _uorb_mutex_take(node, 1)) {
        (*time_sub).time = node->timestamp;
        _uorb_mutex_release(node);
        return UORB_OK;
    }
    return UORB_ERROR;
}

// /**
//  * copy sorb metadata to buffer if meta-data is updated
//  * @param  meta     metadata struct pointer
//  * @param  time_sub subscrible time-handle
//  * @param  buff     copy target buffer
//  * @return          UORB_OK or UORB_ERROR
//  */
// int copy_if_update(struct uorb_metadata *meta, uorb_sub_t *time_sub, void *buff)
// {
//     if (meta == NULL) {
//         return UORB_ERROR;
//     }

//     if (meta->timestamp != time_sub->time || meta->generation != time_sub->generation)
//     {
//         return uorb_copy(meta, time_sub, buff);
//     } else {
//         return UORB_ERROR;
//     }
// }
