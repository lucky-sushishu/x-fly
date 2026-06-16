#include "ist8310.h"

#include "i2c.h"

ist8310_t ist8310;

int ist8310_read_bytes(uint8_t reg_addr, uint8_t *reg_data, uint8_t length)
{
	return i2c_read((IST8310_ADDR << 1) | 0x00, reg_addr, reg_data, length);
}

int ist8310_write_bytes(uint8_t reg_addr, const uint8_t *data, uint8_t data_length)
{
	return i2c_write((IST8310_ADDR << 1) | 0x00, reg_addr, data, data_length);
}

int ist8310_init(void)
{
	int result = 0;
	uint8_t ist8310_device_id = 0;
	ist8310_read_bytes(IST8310_WHO_AM_I, &ist8310_device_id, 1);

	if (ist8310_device_id != IST8310_DEVICE_ID)
	{
		result += 1;
	}

	uint8_t reg = 0, data = 0;
	reg = 0xC0;
	ist8310_write_bytes(IST8310_PDCNTL, &reg, 1);
	ist8310_read_bytes(IST8310_PDCNTL, &data, 1);
	if (reg != data)
	{
		result += 1;
	}

	reg = 0x24;
	ist8310_write_bytes(IST8310_AVGCNTL, &reg, 1);
	ist8310_read_bytes(IST8310_AVGCNTL, &data, 1);
	if (reg != data)
	{
		result += 1;
	}

	return result;
}

int ist8310_update_data(void)
{
	uint8_t data = 0x01;
	ist8310_write_bytes(IST8310_CNTL1, &data, 0x01);

	uint8_t status_register_1 = 0;
	int count = 10;
	while (count-- > 0)
	{
		ist8310_read_bytes(IST8310_STATUS_1, &status_register_1, 1);
		if (status_register_1 & IST8310_STATUS_1_DRDY)
		{
			uint8_t origin_data[6] = {0};
			ist8310_read_bytes(IST8310_DATAXL, &origin_data[0], 6);

			ist8310.data[0] = (int16_t)(origin_data[0] << 8) | origin_data[1];
			ist8310.data[1] = (int16_t)(origin_data[2] << 8) | origin_data[3];
			ist8310.data[2] = (int16_t)(origin_data[4] << 8) | origin_data[5];

			return 0;
		}
		HAL_Delay(1);
	}

	return 1;
}
