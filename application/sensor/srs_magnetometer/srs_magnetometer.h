/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#ifndef SRS_MAGNETOMETER_H
#define SRS_MAGNETOMETER_H

#include "osapi.h"

#define SRS_MAGNETOMETER_REAL_PRIO      20
#define SRS_MAGNETOMETER_STACKSIZE      2048
extern OS_THREAD                        g_srs_magnetometer_tcb;
extern UCHAR                            g_srs_magnetometer_stack[SRS_MAGNETOMETER_STACKSIZE];

void srs_magnetometer_entry(ULONG srs_magnetometer_input);

#endif
