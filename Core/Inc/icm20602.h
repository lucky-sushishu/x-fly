#ifndef ICM20602_H
#define ICM20602_H

#include <stdint.h>

#define ICM20602_PWR_MGMT_1 0x6B
#define ICM20602_WHO_AM_I 	0x75


int icm20602_read(uint8_t reg, uint8_t *reg_data);
int icm20602_write(uint8_t reg, const uint8_t reg_data);

#endif /* !ICM20602_H */
