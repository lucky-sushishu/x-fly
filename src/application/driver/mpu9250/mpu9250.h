#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32f4xx_hal.h"
#include "tx_api.h"
#include "bsp_software_i2c.h"

#define MPU9250_ADDR 0X69 // MPU9250的器件IIC地址
#define MPU9250_ID 0X71   // MPU9250的器件ID

#define MAG_ADDR AK8963_ADDR
#define AK8963_ADDR 0X0C // 内部磁力计AK8963的器件ID的IIC地址
#define AK8963_ID 0X48   // AK8963的器件ID

// AK8963的寄存器地址
#define MAG_WHO_AM_I 0x00 // 器件ID地址
#define MAG_CNTL1 0X0A    // 控制寄存器1
#define MAG_CNTL2 0X0B    // 控制寄存器2

#define MAG_ST1  0x02
#define MAG_HXL  0X03
#define MAG_HXH  0X04
#define MAG_HYL  0X05
#define MAG_HYH  0X06
#define MAG_HZL  0X07
#define MAG_HZH  0X08
#define MAG_ST2  0x09
#define MAG_ASAX 0x10
#define MAG_ASAY 0x11
#define MAG_ASAZ 0x12

// MPU9250的寄存器地址
#define MPU_SAMPLE_RATE_REG 0X19 // 采样频率寄存器
#define MPU_CFG_REG 0X1A         // 配置寄存器
#define MPU_GYRO_CFG_REG 0X1B    // 陀螺仪配置寄存器
#define MPU_ACCL_CFG_REG 0X1C    // 加速度计配置寄存器

#define MPU_FIFO_EN_REG 0X23 // FIFO使能寄存器

#define MPU_INTBP_CFG_REG 0X37 // 中断/旁路设置寄存器
#define MPU_INT_EN_REG 0X38    // 中断使能寄存器
#define MPU_INT_STA_REG 0X3A   // 中断状态寄存器

#define MPU_ACCL_XOUT_H 0X3B  // 加速度计X轴寄存器高8位
#define MPU_ACCL_XOUT_L 0X3C  // 加速度计X轴寄存器低8位
#define MPU_ACCL_YOUT_H 0X3D  // 加速度计Y轴寄存器高8位
#define MPU_ACCL_YOUT_L 0X3E  // 加速度计Y轴寄存器低8位
#define MPU_ACCL_ZOUT_H 0X3F  // 加速度计Z轴寄存器高8位
#define MPU_ACCL_ZOUT_L 0X40  // 加速度计Z轴寄存器低8位
#define MPU_TEMP_OUTH_REG 0X41 // 温度寄存器高8位
#define MPU_TEMP_OUTL_REG 0X42 // 温度寄存器低8位
#define MPU_GYRO_XOUT_H 0X43   // 陀螺仪X轴寄存器高8位
#define MPU_GYRO_XOUT_L 0X44   // 陀螺仪X轴寄存器低8位
#define MPU_GYRO_YOUT_H 0X45   // 陀螺仪Y轴寄存器高8位
#define MPU_GYRO_YOUT_L 0X46   // 陀螺仪Y轴寄存器低8位
#define MPU_GYRO_ZOUT_H 0X47   // 陀螺仪Z轴寄存器高8位
#define MPU_GYRO_ZOUT_L 0X48   // 陀螺仪Z轴寄存器低8位

#define MPU_USER_CTRL_REG 0X6A // 用户控制寄存器
#define MPU_PWR_MGMT1_REG 0X6B // 电源管理寄存器1
#define MPU_PWR_MGMT2_REG 0X6C // 电源管理寄存器2

#define MPU_WHO_AM_I 0X75 // 器件ID寄存器

#define MPU9250_ACCL_16G_SEN 0.00478515625f

#define MPU9250_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define MPU9250_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define MPU9250_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define MPU9250_GYRO_250_SEN 0.00013315805450396191230191732547673f

typedef struct mpu9250_raw_data_s
{
    int16_t x;
    int16_t y;
    int16_t z;
} mpu9250_raw_data_t;

typedef struct mpu9250_data_s
{
    float data[3];
} mpu9250_data_t;


uint8_t mpu_set_gyro_range(uint8_t range);
uint8_t mpu_set_accl_range(uint8_t range);

uint8_t mpu9250_init(void);

int16_t mpu9250_get_temperture(void);
uint8_t mpu9250_get_gyro(mpu9250_data_t *buf);
uint8_t mpu9250_get_accl(mpu9250_data_t *buf);
uint8_t mpu9250_get_mag(mpu9250_data_t *buf);

#endif
