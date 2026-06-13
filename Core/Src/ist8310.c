#include "ist8310.h"

#include "i2c.h"

int ist8310_read_byte(uint8_t reg_addr, uint8_t *reg_data)
{
	return HAL_I2C_Mem_Read(&hi2c1, (IST8310_ADDR << 1) | 0x00, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, 1, 5);
}

int ist8310_write_byte(uint8_t reg_addr, const uint8_t reg_data)
{
	return HAL_I2C_Mem_Write(&hi2c1, (IST8310_ADDR << 1) | 0x00, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg_data, 1, 5);
}
