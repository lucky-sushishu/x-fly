#include "icm20689.h"

/* include spi read & write file */
#include "spi.h"

int icm20689_read(uint8_t reg, uint8_t *reg_data)
{
	uint8_t tx_data[2] = {0};
	uint8_t rx_data[2] = {0};
	tx_data[0] = reg | 0x80;
	spi_write_read(SPI3_CS4, &tx_data[0], &rx_data[0], 2);

	*reg_data = rx_data[1];

	return 0;
}

int icm20689_write(uint8_t reg, const uint8_t reg_data)
{
	return 0;
}
