#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>

/*******************************************************************************
 * MPU-6050 硬件引脚定义 (软件模拟 I2C)
 * PB10: SCL
 * PB11: SDA
 * AD0 = 0 (接地), I2C 地址 = 0x68
 *******************************************************************************/
#define MPU6050_ADDR         0x68

#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_INT_ENABLE   0x38
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_TEMP_OUT_H   0x41
#define MPU_REG_GYRO_XOUT_H  0x43
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75  // 固定返回值: 0x68

uint8_t MPU6050_Init(void);
uint8_t MPU6050_Read_ID(void);
uint8_t MPU6050_Read_Raw(int16_t accel[3], int16_t gyro[3]);
uint8_t MPU6050_Get_Data(float accel_g[3], float gyro_dps[3]);

#endif /* __MPU6050_H__ */
