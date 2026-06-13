#include "icm20602.h"

/* include spi read & write file */
#include "spi.h"

int icm20602_read(uint8_t reg, uint8_t *reg_data)
{
	uint8_t tx_data[2] = {0};
	uint8_t rx_data[2] = {0};
	tx_data[0] = reg | 0x80;
	spi_write_read(SPI3_CS3, &tx_data[0], &rx_data[0], 2);

	*reg_data = rx_data[1];

	return 0;
}

int icm20602_write(uint8_t reg, const uint8_t reg_data)
{
	uint8_t data[2] = {0};
	/*  first btye's bit 0 set 0, it's mean write */
	data[0] = reg & 0x7F;
	data[1] = reg_data;
	spi_write(SPI3_CS3, &data[0], 2);
	return 0;
}
