#include "ms5611.h"
#include "i2c.h"

#include <math.h>

ms5611_t ms5611;

int ms5611_read_bytes(uint8_t reg_addr, uint8_t *reg_data, uint8_t length)
{
	return i2c_read((MS5611_ADDR << 1) | 0x00, reg_addr, reg_data, length);
}

int ms5611_receive_bytes(uint8_t *data, uint8_t length)
{
	return i2c_receive((MS5611_ADDR << 1) | 0x00, data, length);
}

int ms5611_transmit_bytes(const uint8_t *data, uint8_t length)
{
	return i2c_transmit((MS5611_ADDR << 1) | 0x00, data, length);
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
	ms5611_transmit_bytes(&cmd, 1);
}

int ms5611_update_pressure(void)
{
	int i = 0;
	uint8_t cmd = 0;

	/* Initiate a pressure conversion */
	cmd = MS5611_INITIATE_PRESSURE_CONVERSION;
	i += ms5611_transmit_bytes(&cmd, 1);

	/* Wait conversion is finished */
	HAL_Delay(10);

	/* ADC read sequence */
	cmd = MS5611_ADC_READ_SEQUENCE;
	i += ms5611_transmit_bytes(&cmd, 1);

	/* Read Answer */
	i += ms5611_receive_bytes(&ms5611.origin_pressure[0], 3);

	uint32_t d1 	= (uint32_t)ms5611.origin_pressure[0] << 16 | (uint32_t)ms5611.origin_pressure[1] << 8 | (uint32_t)ms5611.origin_pressure[2];
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
	i += ms5611_transmit_bytes(&cmd, 1);

	/* Wait conversion is finished */
	HAL_Delay(10);

	/* ADC read sequence */
	cmd = MS5611_ADC_READ_SEQUENCE;
	i += ms5611_transmit_bytes(&cmd, 1);

	/* Read Answer */
	i += ms5611_receive_bytes(&ms5611.origin_temperature[0], 3);

	/* Calculate temperature */
	uint32_t d2 	   	= (uint32_t)ms5611.origin_temperature[0] << 16 | (uint32_t)ms5611.origin_temperature[1] << 8 | (uint32_t)ms5611.origin_temperature[2];
	ms5611.dt 	 	   	= (int32_t)d2 - (int32_t)ms5611.prom[4] * (1 << 8);
	ms5611.temp		   	= 2000 + (int64_t)ms5611.dt * ms5611.prom[5] / (1 << 23);

	/* SECOND ORDER TEMPERATURE COMPENSATION */
	if (ms5611.temp >= 2000)
	{
		ms5611.t2    = 0;
		ms5611.off2  = 0;
		ms5611.sens2 = 0;
	}
	else if (ms5611.temp < 2000)
	{
		ms5611.t2    = (int64_t)ms5611.dt * ms5611.dt / (1 << 31);
		ms5611.off2  = 5 * pow((ms5611.temp - 2000), 2) / (1 << 1);
		ms5611.sens2 = 5 * pow((ms5611.temp - 2000), 2) / (1 << 2);

		if (ms5611.temp < -1500)
		{
			ms5611.off2  += 7 * pow((ms5611.temp + 1500), 2);
			ms5611.sens2 += 11 * pow((ms5611.temp + 1500), 2) / (1 << 2);
		}
	}

	ms5611.temp = ms5611.temp - ms5611.t2;
	ms5611.off  = ((int64_t)ms5611.prom[1] * (1 << 16) + ((int64_t)ms5611.prom[3] * ms5611.dt) / (1 << 7) - ms5611.off2);
	ms5611.sens = ((int64_t)ms5611.prom[0] * (1 << 15) + ((int64_t)ms5611.prom[2] * ms5611.dt) / (1 << 8) - ms5611.sens2);

	ms5611.temperature  = ms5611.temp / 100.0f;

	ms5611.barometer.temperature = ms5611.temperature;

	return i;
}
