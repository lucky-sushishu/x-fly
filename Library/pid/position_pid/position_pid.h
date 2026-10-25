/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#ifndef POSITION_PID_H
#define POSITION_PID_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct position_pid_s position_pid_t;

struct position_pid_s
{
    float aim_altitude;
    float present_altitude;
    float err;          //比例值，当前误差
    float err_last;
    float i_past;       //积分值，过去积累的误差
    float d_future;     //微分值，未来可能的误差
    float p;
    float i;
    float d;
    float output;
};

void  position_pid_init(void);
float position_pid_update(float aim);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_PID_H */
