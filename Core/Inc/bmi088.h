#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>
#include "imu.h"

typedef struct bmi088_s bmi088_t;
typedef enum bmi088_sensor_type_e bmi088_sensor_type_t;

struct bmi088_s
{
	int16_t origin_accl_data[3];
	int16_t origin_gyro_data[3];

	imu_t   imu;
};

enum bmi088_sensor_type_e
{
	BMI088_ACCL,
	BMI088_GYRO,
};

/* bmi088 reg addr list */
#define BMI088_ACCL_CHIP_ID		        0x00
#define BMI088_ACCL_DATA		  		0x12
#define BMI088_ACCL_CHIP_ID_VALUE		0x1E
#define BMI088_ACCL_RANGE				0x41
#define BMI088_ACCL_RANGE_12G			0x02
#define BMI088_ACCL_RANGE_24G			0x03
#define BMI088_ACCL_PWR_CONF		 	0x7C
#define BMI088_ACCL_PWR_CTRL		  	0x7D

#define BMI088_GYRO_CHIP_ID		        0x00
#define BMI088_GYRO_DATA		  		0x02
#define BMI088_GYRO_CHIP_ID_VALUE		0x0F
#define BMI088_GYRO_RANGE				0x0F
#define BMI088_GYRO_RANGE_2000			0x00

/* bmi088 reg value set */
#define BMI088_ACC_PWR_CTRL_ACC_ENABLE  0x04
#define BMI088_ACC_PWR_CTRL_ACC_DISABLE	0x00

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f


#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

extern bmi088_t bmi088;

int bmi088_read(bmi088_sensor_type_t sensor_type, uint8_t reg, uint8_t *reg_data);
int bmi088_write(bmi088_sensor_type_t sensor_type, uint8_t reg, const uint8_t reg_data);

int bmi088_init(void);
int bmi088_update_data(void);

#endif /* !BMI088_H */
