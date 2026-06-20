/******************************************************************************/
/* Copyright (c) 2026 Star Ring Studio. All rights reserved.                  */
/*                                                                            */
/* This file is part of the Star Ring Studio (SRS) project and is licensed    */
/* under the terms specified in the LICENSE file included with this project.  */
/*                                                                            */
/* Unauthorized copying of this file, via any medium, is strictly prohibited. */
/* Proprietary and confidential.                                              */
/******************************************************************************/

#ifndef SRS_BAROMETER_H
#define SRS_BAROMETER_H

#include "osapi.h"

#define SRS_BAROMETER_REAL_PRIO     20
#define SRS_BAROMETER_STACKSIZE     2048
extern OS_THREAD                    g_srs_barometer_tcb;
extern UCHAR                        g_srs_barometer_stack[SRS_BAROMETER_STACKSIZE];

void srs_barometer_entry(ULONG srs_barometer_input);

#endif
