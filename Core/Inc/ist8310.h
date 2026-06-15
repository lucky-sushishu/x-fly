#ifndef IST8310_H
#define IST8310_H

#include <stdint.h>

#define IST8310_ADDR			0x0E
#define IST8310_DEVICE_ID		0x10

#define IST8310_WHO_AM_I		0x00

#define IST8310_STATUS_1		0x02

#define IST8310_DATAXL			0x03
#define IST8310_DATAXH			0x04
#define IST8310_DATAYL			0x05
#define IST8310_DATAYH			0x06
#define IST8310_DATAZL			0x07
#define IST8310_DATAZH			0x08

#define IST8310_CNTL1			0x0A

#define IST8310_AVGCNTL			0x41
#define IST8310_PDCNTL			0x42

#define IST8310_STATUS_1_DRDY	0x01
#define IST8310_STATUS_1_DOR	0x02


typedef struct ist8310_s ist8310_t;

struct ist8310_s
{
	uint8_t origin_data[6];
};

extern ist8310_t ist8310;

int ist8310_read_bytes(uint8_t reg_addr, uint8_t *reg_data, uint8_t length);
int ist8310_write_bytes(uint8_t reg_addr, const uint8_t *data, uint8_t data_length);

int ist8310_init(void);
int ist8310_update_data(void);

#endif
