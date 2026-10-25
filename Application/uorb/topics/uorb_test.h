#pragma once
#ifndef UORB_TEST_H
#define UORB_TEST_H

#include "osapi-uorb.h"


#ifdef __cplusplus
typedef struct  __EXPORT uorb_test_s {
#else
typedef struct uorb_test_s {
#endif
	bool updated;
	uint64_t timestamp;

} uorb_test_t;

UORB_DECLARE(uorb_test);

#endif
