#ifndef ICM20689_H
#define ICM20689_H

#include <stdint.h>

#define ICM20689_WHO_AM_I 0x75


int icm20689_read(uint8_t reg, uint8_t *reg_data);
int icm20689_write(uint8_t reg, const uint8_t reg_data);

#endif /* !ICM20689_H */
