#include "ms5611.h"
#include "i2c.h"

ms5611_t ms5611;

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
		ms5611.prom[i] = (uint16_t)data[0] << 8 | data[1];
	}


	return 0;
}

void ms5611_reset(void)
{
	uint8_t cmd = MS5611_RESET;
	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR << 1), &cmd, I2C_MEMADD_SIZE_8BIT, 5);
}

int ms5611_update_pressure(void)
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
	i += HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR << 1) | 0x00, &ms5611.origin_pressure[0], 3, 5);

	uint32_t d1 	= (uint32_t)ms5611.origin_pressure[0] << 16 | (uint32_t)ms5611.origin_pressure[1] << 8 | (uint32_t)ms5611.origin_pressure[2];
	ms5611.off  	= (int64_t)ms5611.prom[1] * (1 << 16) + ((int64_t)ms5611.prom[3] * ms5611.dt) / (1 << 7);
	ms5611.sens 	= (int64_t)ms5611.prom[0] * (1 << 15) + ((int64_t)ms5611.prom[2] * ms5611.dt) / (1 << 8);
	ms5611.pressure = ((d1 * ms5611.sens / (1 << 21) - ms5611.off) / (1 << 15)) / 100.0f;

	ms5611.barometer.pressure = ms5611.pressure;

	return i;
}

int ms5611_update_temperature(void)
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
	i += HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR << 1) | 0x00, &ms5611.origin_temperature[0], 3, 5);

	/* Calculate temperature */
	uint32_t d2 	   	= (uint32_t)ms5611.origin_temperature[0] << 16 | (uint32_t)ms5611.origin_temperature[1] << 8 | (uint32_t)ms5611.origin_temperature[2];
	ms5611.dt 	 	   	= (int32_t)d2 - (int32_t)ms5611.prom[4] * (1 << 8);
	ms5611.temp		   	= 2000 + (int64_t)ms5611.dt * ms5611.prom[5] / (1 << 23);
	ms5611.temperature  = ms5611.temp / 100.0f;

	ms5611.barometer.temperature = ms5611.temperature;

	return i;
}
