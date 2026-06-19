#include "bmi088.h"

/* include spi read & write file */
#include "spi.h"

#include <math.h>

bmi088_t bmi088;

int bmi088_read(bmi088_sensor_type_t sensor_type, uint8_t reg, uint8_t *reg_data)
{
	spi_cs_t cs = (sensor_type == BMI088_ACCL) ? SPI3_CS1 : SPI3_CS2;

	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	/*  first btye's bit 0 set 1, it's mean read */
	tx_data[0] = reg | 0x80;
	spi_write_read(cs, &tx_data[0], &rx_data[0], 3);

	*reg_data = (sensor_type == BMI088_ACCL) ? rx_data[2] : rx_data[1];
	return 0;
}

int bmi088_write(bmi088_sensor_type_t sensor_type, uint8_t reg, const uint8_t reg_data)
{
	spi_cs_t cs = (sensor_type == BMI088_ACCL) ? SPI3_CS1 : SPI3_CS2;

	uint8_t data[2] = {0};
	/*  first btye's bit 0 set 0, it's mean write */
	data[0] = reg & 0x7F;
	data[1] = reg_data;
	spi_write(cs, &data[0], 2);

	return 0;
}

int bmi088_read_bytes(bmi088_sensor_type_t sensor_type, uint8_t reg, uint8_t *data, uint8_t data_length)
{
	int i = 0;
	while (data_length--)
	{
		bmi088_read(sensor_type, reg++, &data[i++]);
	}

	return 0;
}

int bmi088_init(void)
{
	int result = 0;
	uint8_t bmi088_accl_range = 0;
	uint8_t bmi088_gyro_range = 0;
	uint8_t bmi088_accl_chip_id = 0;
	uint8_t bmi088_gyro_chip_id = 0;

	bmi088_write(BMI088_ACCL, BMI088_ACCL_PWR_CTRL, BMI088_ACC_PWR_CTRL_ACC_ENABLE);
	HAL_Delay(1);

	bmi088_write(BMI088_ACCL, BMI088_ACCL_RANGE, BMI088_ACCL_RANGE_24G);
	HAL_Delay(1);
	bmi088_read(BMI088_ACCL, BMI088_ACCL_RANGE, &bmi088_accl_range);
	HAL_Delay(1);
	if (bmi088_accl_range != BMI088_ACCL_RANGE_24G) {
		result += 1;
	}
	bmi088_write(BMI088_GYRO, BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_2000);
	HAL_Delay(1);
	bmi088_read(BMI088_GYRO, BMI088_GYRO_RANGE, &bmi088_gyro_range);
	if (bmi088_gyro_range != BMI088_GYRO_RANGE_2000) {
		result += 1;
	}
	HAL_Delay(1);

	bmi088_read(BMI088_ACCL, BMI088_ACCL_CHIP_ID, &bmi088_accl_chip_id);
	HAL_Delay(1);
	bmi088_read(BMI088_GYRO, BMI088_GYRO_CHIP_ID, &bmi088_gyro_chip_id);
	HAL_Delay(1);

	if (!(bmi088_accl_chip_id == BMI088_ACCL_CHIP_ID_VALUE &&
		  bmi088_gyro_chip_id == BMI088_GYRO_CHIP_ID_VALUE))
	{
		result += 1;
	}

	return result;
}

int bmi088_update_data(void)
{
	uint8_t accl_data[6] = {0};
	uint8_t gyro_data[6] = {0};

	bmi088_read_bytes(BMI088_ACCL, BMI088_ACCL_DATA, &accl_data[0], 6);
	bmi088_read_bytes(BMI088_GYRO, BMI088_GYRO_DATA, &gyro_data[0], 6);

	bmi088.origin_accl_data[0] = (int16_t)accl_data[1] << 8 | accl_data[0];
	bmi088.origin_accl_data[1] = (int16_t)accl_data[3] << 8 | accl_data[2];
	bmi088.origin_accl_data[2] = (int16_t)accl_data[5] << 8 | accl_data[4];
	bmi088.origin_gyro_data[0] = (int16_t)gyro_data[1] << 8 | gyro_data[0];
	bmi088.origin_gyro_data[1] = (int16_t)gyro_data[3] << 8 | gyro_data[2];
	bmi088.origin_gyro_data[2] = (int16_t)gyro_data[5] << 8 | gyro_data[4];

	// bmi088.imu.accl[0]         = (float)bmi088.origin_accl_data[0] / 32768 * 1000 * pow(2, BMI088_ACCL_RANGE_24G + 1) * 1.5f;
	// bmi088.imu.accl[1]         = (float)bmi088.origin_accl_data[1] / 32768 * 1000 * pow(2, BMI088_ACCL_RANGE_24G + 1) * 1.5f;
	// bmi088.imu.accl[2]         = (float)bmi088.origin_accl_data[2] / 32768 * 1000 * pow(2, BMI088_ACCL_RANGE_24G + 1) * 1.5f;
	bmi088.imu.accl[0]         = bmi088.origin_accl_data[0] * BMI088_ACCEL_24G_SEN;
	bmi088.imu.accl[1]         = bmi088.origin_accl_data[1] * BMI088_ACCEL_24G_SEN;
	bmi088.imu.accl[2]         = bmi088.origin_accl_data[2] * BMI088_ACCEL_24G_SEN;

	// bmi088.imu.gyro[0]         = (float)bmi088.origin_gyro_data[0] / 32768 * 2000.0f;
	// bmi088.imu.gyro[1]         = (float)bmi088.origin_gyro_data[1] / 32768 * 2000.0f;
	// bmi088.imu.gyro[2]         = (float)bmi088.origin_gyro_data[2] / 32768 * 2000.0f;
	bmi088.imu.gyro[0]         = (float)bmi088.origin_gyro_data[0] * BMI088_GYRO_2000_SEN;
	bmi088.imu.gyro[1]         = (float)bmi088.origin_gyro_data[1] * BMI088_GYRO_2000_SEN;
	bmi088.imu.gyro[2]         = (float)bmi088.origin_gyro_data[2] * BMI088_GYRO_2000_SEN;

	return 0;
}
