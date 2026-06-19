#include "ist8310.h"

#include "i2c.h"

ist8310_t ist8310;

#define IST8310_WRITE_REG_NUM 4 

static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] ={
        {0x0B, 0x08, 0x01},     //enalbe interrupt  and low pin polarity.开启中断，并且设置低电平
        {0x41, 0x09, 0x02},     //average 2 times.平均采样两次
        {0x42, 0xC0, 0x03},     //must be 0xC0. 必须是0xC0
        {0x0A, 0x0B, 0x04}};    //200Hz output rate.200Hz输出频率


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

	uint8_t write_num;
	for (write_num = 0; write_num < IST8310_WRITE_REG_NUM; write_num++)
    {
		uint8_t reg_data = 0;
        ist8310_write_bytes(ist8310_write_reg_data_error[write_num][0], &ist8310_write_reg_data_error[write_num][1], 1);
        ist8310_read_bytes(ist8310_write_reg_data_error[write_num][0], &reg_data, 1);
        if (reg_data != ist8310_write_reg_data_error[write_num][1])
        {
            return ist8310_write_reg_data_error[write_num][2];
        }
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

			ist8310.data[0] = (int16_t)(origin_data[1] << 8) | origin_data[0];
			ist8310.data[1] = (int16_t)(origin_data[3] << 8) | origin_data[2];
			ist8310.data[2] = (int16_t)(origin_data[5] << 8) | origin_data[4];

			ist8310.magnetometer.mag[0] = MAG_SEN * ist8310.data[0];
			ist8310.magnetometer.mag[1] = MAG_SEN * ist8310.data[1];
			ist8310.magnetometer.mag[2] = MAG_SEN * ist8310.data[2];

			return 0;
		}
		HAL_Delay(1);
	}

	return 1;
}
