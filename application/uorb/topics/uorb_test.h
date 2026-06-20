
#include "osapi-uorb.h"

#pragma once

#ifdef __cplusplus
typedef struct  __EXPORT uorb_test_s {
#else
typedef struct uorb_test_s {
#endif
	bool updated;
	uint64_t timestamp;

}uorb_test_t;

UORB_DECLARE(uorb_test);
