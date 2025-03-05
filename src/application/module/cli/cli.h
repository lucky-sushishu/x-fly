#ifndef _CLI_H_
#define _CLI_H_

#include "includes.h"

#define CLI_PRIO 20
#define CLI_STACKSIZE 4096
extern TX_THREAD cli_tcb;
extern UCHAR cli_stack[CLI_STACKSIZE];

void cli_entry(ULONG thread_input);

#endif
