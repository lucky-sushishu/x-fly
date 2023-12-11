#include "mpu9250.h"

TX_SEMAPHORE            tx_sdriver_input_semaphore;
TX_SEMAPHORE            tx_sdriver_output_semaphore;

VOID tx_sdriver_initialize(VOID)
{
	/* Initialize the two counting semaphores used to control
	the simple driver I/O. */
	tx_semaphore_create(&tx_sdriver_input_semaphore,
											"simple driver input semaphore", 0);
	tx_semaphore_create(&tx_sdriver_output_semaphore,
											"simple driver output semaphore", 1);

	/* Setup interrupt vectors for input and output ISRs.
	The initial vector handling should call the ISRs
	defined in this file. */

	/* Configure serial device hardware for RX/TX interrupt
	generation, baud rate, stop bits, etc. */
	i2c_init();
}
