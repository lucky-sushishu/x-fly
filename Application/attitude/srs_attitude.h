/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#ifndef SRS_ATTITUDE_H
#define SRS_ATTITUDE_H

#include "osapi.h"

#define SRS_ATTITUDE_REAL_PRIO      20
#define SRS_ATTITUDE_STACKSIZE      4096
extern OS_THREAD                    g_srs_attitude_tcb;
extern UCHAR                        g_srs_attitude_stack[SRS_ATTITUDE_STACKSIZE];

#define DEG2RAD 0.01745329252f

typedef enum status_e status_t;
typedef struct attitude_s attitude_t;

enum status_e
{
    INIT,
    WORK,
};

struct attitude_s
{
    float gx;   /* 单位：弧度 */
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;

    float roll;
    float pitch;
    float yaw;
};

void srs_attitude_entry(ULONG srs_attitude_input);

#endif
