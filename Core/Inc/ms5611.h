#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>
#include "barometer.h"

#define MS5611_ADDR								0x77

#define MS5611_PROM_ADDR						0xA2

#define MS5611_RESET							0x1E
#define MS5611_INITIATE_PRESSURE_CONVERSION 	0x48
#define MS5611_INITIATE_TEMPERATURE_CONVERSION 	0x58
#define MS5611_ADC_READ_SEQUENCE 				0x00

/* MS5611主体 */
typedef struct ms5611_s ms5611_t;

struct ms5611_s
{
    uint16_t    prom[6];                /* C1~C6 */

    int32_t		dt;						/* DT */
    int32_t		temp;					/* TEMP */
    int64_t		off;					/* OFF */
    int64_t		sens;					/* SENS */
    int64_t		t2;						/* T2 */
    int64_t		off2;					/* OFF2 */
    int64_t		sens2;					/* SENS2 */

    uint8_t     origin_pressure[3];     /* 原始气压数据 */
    uint8_t     origin_temperature[3];  /* 原始温度数据 */

    float       pressure;               /* 直接由原始数据转换的气压数据 */
    float       temperature;            /* 直接由原始数据转换的温度数据 */

    barometer_t barometer;              /* 由温度补偿计算出的气压数据 */
};

extern ms5611_t ms5611;

int ms5611_init(void);
void ms5611_reset(void);
int ms5611_update_pressure(void);
int ms5611_update_temperature(void);

#endif
