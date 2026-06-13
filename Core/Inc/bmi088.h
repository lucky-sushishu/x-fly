#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>

typedef enum bmi088_sensor_type_e bmi088_sensor_type_t;

enum bmi088_sensor_type_e
{
	BMI088_ACCE,
	BMI088_GYRO,
};

/* bmi088 reg addr list */
#define BMI088_ACCE_CHIP_ID		        0x00
#define BMI088_ACCE_PWR_CONF		 	0x7C
#define BMI088_ACCE_PWR_CTRL		  	0x7D

#define BMI088_GYRO_CHIP_ID		        0x00

/* bmi088 reg value set */
#define BMI088_ACC_PWR_CTRL_ACC_ENABLE  0x04
#define BMI088_ACC_PWR_CTRL_ACC_DISABLE	0x00

int bmi088_read(bmi088_sensor_type_t sensor_type, uint8_t reg, uint8_t *reg_data);
int bmi088_write(bmi088_sensor_type_t sensor_type, uint8_t reg, const uint8_t reg_data);

#endif /* !BMI088_H */
