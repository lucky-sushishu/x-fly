#ifndef IST8310_H
#define IST8310_H

#include <stdint.h>

#define IST8310_ADDR		0x0E
#define IST8310_DEVICE_ID	0x10

#define IST8310_WHO_AM_I	0x00

int ist8310_read_byte(uint8_t reg_addr, uint8_t *reg_data);
int ist8310_write_byte(uint8_t reg_addr, const uint8_t reg_data);

#endif
