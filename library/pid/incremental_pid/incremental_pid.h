/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#ifndef INCREMENTAL_PID_H
#define INCREMENTAL_PID_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct incremental_pid_s incremental_pid_t;

struct incremental_pid_s
{
	float set_speed;        //定义设定值
	float actual_speed;     //定义实际值
	float err;              //定义偏差值
	float err_next;         //定义上一个偏差值
	float err_last;         //定义上上一个偏差值
	float kp, ki, kd;       //定义比例，积分，微分比例
};

void  incremental_pid_init(void);
float incremental_pid_update(float speed);

#ifdef __cplusplus
}
#endif

#endif /* INCREMENTAL_PID_H */
