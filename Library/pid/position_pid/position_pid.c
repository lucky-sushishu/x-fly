/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/
#include "position_pid.h"

/*
位置式pid
*/
static position_pid_t pid;

void position_pid_init(void)
{
    pid.aim_altitude    = 0;
    pid.present_altitude= 0;
    pid.err             = 0;
    pid.err_last        = 0;
    pid.i_past          = 0;
    pid.d_future        = 0;
    pid.p               = 0.2;
    pid.i               = 0.015;
    pid.d               = 0.2;
    pid.output          = 0;
}

float position_pid_update(float aim)
{
    pid.aim_altitude     =  aim;
    pid.err              =  pid.aim_altitude-pid.present_altitude;              //比例值当前误差
    pid.i_past           += pid.err;                                            //积分值，过去积累的误差
    pid.d_future         =  pid.err-pid.err_last;                               //微分值，未来可能出现的误差
    pid.output           =  pid.p*pid.err+pid.i*pid.i_past+pid.d*pid.d_future;  //最重要的是这个公式
    pid.err_last         =  pid.err;
    pid.present_altitude =  pid.output*1.0;                                     //输出转化为当前值
    return pid.present_altitude;
}

// int main(int argc, char *argv)
// {
// 	int i_count=0;
//     position_pid_init();
//     while (i_count<1000)
//     {
//         float altitude=position_pid_update(100.0);
//         printf("%f\n",altitude);
//         i_count++;
//     }
// }
