#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>

#define MS5611_ADDR								0x77

#define MS5611_PROM_ADDR						0xA2

#define MS5611_RESET							0x1E
#define MS5611_INITIATE_PRESSURE_CONVERSION 	0x48
#define MS5611_INITIATE_TEMPERATURE_CONVERSION 	0x58
#define MS5611_ADC_READ_SEQUENCE 				0x00

/* Pressure */

/* Temperature */

extern uint16_t g_ms5611_prom[6];

int ms5611_read_bytes(uint8_t reg_addr, uint8_t *reg_data, uint8_t length);
int ms5611_write_byte(uint8_t reg_addr, const uint8_t reg_data);

int ms5611_init(void);
void ms5611_reset(void);
int ms5611_read_pressure_value(uint8_t *value);
int ms5611_read_temperature_value(uint8_t *value);

#endif
