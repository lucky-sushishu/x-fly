/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#include "incremental_pid.h"

/*
增量式pid
*/
static incremental_pid_t pid;

void incremental_pid_init(void)
{
	pid.set_speed 	 = 0.0;
	pid.actual_speed = 0.0;
	pid.err 		 = 0.0;
	pid.err_last 	 = 0.0;
	pid.err_next 	 = 0.0;
	pid.kp 			 = 0.2;
	pid.ki 			 = 0.04;
	pid.kd 			 = 0.2;
}

float incremental_pid_update(float speed)
{
	pid.set_speed 		 =  speed;
	pid.err 			 =  pid.set_speed - pid.actual_speed;
	float incrementSpeed =  pid.kp*(pid.err - pid.err_next) + pid.ki*pid.err + pid.kd*(pid.err - 2 * pid.err_next + pid.err_last);
	pid.actual_speed 	 += incrementSpeed;
	pid.err_last 		 =  pid.err_next;
	pid.err_next 		 =  pid.err;
	return pid.actual_speed;
}

// int main()
// {
// 	printf("system begin \n");
// 	incremental_pid_init();
// 	int count = 0;
// 	while (count<1000)
// 	{
// 		float speed = incremental_pid_update(200.0);
// 		printf("%f\n", speed);
// 		count++;
// 	}
// 	return 0;
// }
