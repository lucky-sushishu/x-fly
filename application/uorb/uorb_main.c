
#include "osapi.h"
#include "sys_time.h"

#include "topics/uorb_test.h"
UORB_DEFINE(uorb_test, uorb_test_t);

#include "topics/sensor_imu.h"
UORB_DEFINE(sensor_imu, sensor_imu_t);

#include "topics/sensor_barometer.h"
UORB_DEFINE(sensor_barometer, sensor_barometer_t);

#include "topics/sensor_magnetometer.h"
UORB_DEFINE(sensor_magnetometer, sensor_magnetometer_t);































uint64_t orb_timestamp_get(void)
{
	return sys_absolute_time();
}

INT orb_mutex_create(struct uorb_node_s *node)
{
    node->mtx_lock = OS_MutSemCreateRet("uorb", OS_NO_INHERIT);
	if (node->mtx_lock == NULL)
	{
		return UORB_MUTEX_ERROR;
	}

	return UORB_MUTEX_OK;
}

INT orb_mutex_take(struct uorb_node_s *node, UINT millisec)
{
	if(node->mtx_lock != NULL){
		if(OS_MutSemGet(node->mtx_lock, millisec) != OS_SUCCESS){
			return UORB_MUTEX_ERROR;
		}
	}
	else{
		return UORB_MUTEX_ERROR;
	}

	return UORB_MUTEX_OK;
}

INT	 orb_mutex_release(struct uorb_node_s *node)
{
	if(node->mtx_lock != NULL){
		OS_MutSemPut(node->mtx_lock);
	}
	else{
		return UORB_MUTEX_ERROR;
	}

	return UORB_MUTEX_OK;
}

void orb_delay(UINT millisec)
{
	OS_TaskDelay(1);
}

uint32_t orb_malloc(VOID **memory_ptr, uint32_t size)
{
	return OS_ByteAllocate(&os_uorb_bytepool, memory_ptr, size, OS_NO_WAIT);
}

int uorb_init(void)
{
	uorb_initialize(orb_timestamp_get,
                    orb_mutex_create,
                    orb_mutex_take,
                    orb_mutex_release,
                    orb_delay,
                    orb_malloc);

    return 0;
}
