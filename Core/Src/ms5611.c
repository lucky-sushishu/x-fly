#include "ms5611.h"
#include "i2c.h"

uint16_t g_ms5611_prom[6] = {0};

int ms5611_read_bytes(uint8_t reg_addr, uint8_t *reg_data, uint8_t length)
{
	return HAL_I2C_Mem_Read(&hi2c1, (MS5611_ADDR << 1) | 0x00, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, length, 5);
}

int ms5611_write_byte(uint8_t reg_addr, const uint8_t reg_data)
{
	return HAL_I2C_Mem_Write(&hi2c1, (MS5611_ADDR << 1) | 0x00, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg_data, 1, 5);
}

int ms5611_init(void)
{
	int i;
	uint8_t data[2] = {0};
	ms5611_reset();
	HAL_Delay(10);
	for (i = 0; i < 6; i++)
	{
		ms5611_read_bytes(MS5611_PROM_ADDR + (i * 2), (uint8_t *)&data[0], 2);
		g_ms5611_prom[i] = (uint16_t)data[0] << 8 | data[1];
	}

	return 0;
}

void ms5611_reset(void)
{
	uint8_t cmd = MS5611_RESET;
	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR << 1), &cmd, I2C_MEMADD_SIZE_8BIT, 5);
}

int ms5611_read_pressure_value(uint8_t *value)
{
	int i = 0;
	uint8_t cmd = 0;
	/* Initiate a pressure conversion */
	cmd = MS5611_INITIATE_PRESSURE_CONVERSION;
	i += HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR << 1) | 0x00, &cmd, I2C_MEMADD_SIZE_8BIT, 5);

	/* Wait conversion is finished */
	HAL_Delay(10);

	/* ADC read sequence */
	cmd = MS5611_ADC_READ_SEQUENCE;
	i += HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR << 1) | 0x00, &cmd, I2C_MEMADD_SIZE_8BIT, 5);

	/* Read Answer */
	i += HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR << 1) | 0x00, value, 3, 5);

	return i;
}

int ms5611_read_temperature_value(uint8_t *value)
{
	int i = 0;
	uint8_t cmd = 0;
	/* Initiate a pressure conversion */
	cmd = MS5611_INITIATE_TEMPERATURE_CONVERSION;
	i += HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR << 1) | 0x00, &cmd, I2C_MEMADD_SIZE_8BIT, 5);

	/* Wait conversion is finished */
	HAL_Delay(10);

	/* ADC read sequence */
	cmd = MS5611_ADC_READ_SEQUENCE;
	i += HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR << 1) | 0x00, &cmd, I2C_MEMADD_SIZE_8BIT, 5);

	/* Read Answer */
	i += HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR << 1) | 0x00, value, 3, 5);
	return i;
}
