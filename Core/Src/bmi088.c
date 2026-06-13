#include "bmi088.h"

/* include spi read & write file */
#include "spi.h"

int bmi088_read(bmi088_sensor_type_t sensor_type, uint8_t reg, uint8_t *reg_data)
{
	spi_cs_t cs = (sensor_type == BMI088_ACCE) ? SPI3_CS1 : SPI3_CS2;

	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	/*  first btye's bit 0 set 1, it's mean read */
	tx_data[0] = reg | 0x80;
	spi_write_read(cs, &tx_data[0], &rx_data[0], 3);

	*reg_data = (sensor_type == BMI088_ACCE) ? rx_data[2] : rx_data[1];
	return 0;
}

int bmi088_write(bmi088_sensor_type_t sensor_type, uint8_t reg, const uint8_t reg_data)
{
	spi_cs_t cs = (sensor_type == BMI088_ACCE) ? SPI3_CS1 : SPI3_CS2;

	uint8_t data[2] = {0};
	/*  first btye's bit 0 set 0, it's mean write */
	data[0] = reg & 0x7F;
	data[1] = reg_data;
	spi_write(cs, &data[0], 2);

	return 0;
}
