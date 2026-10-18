#ifndef __LSM6DSR_H__
#define __LSM6DSR_H__

#include <stdint.h>

/*******************************************************************************
 * LSM6DSRTR 寄存器映射
 *******************************************************************************/
#define LSM6DSR_REG_FUNC_CFG_ACCESS    0x01
#define LSM6DSR_REG_PIN_CTRL           0x02
#define LSM6DSR_REG_FIFO_CTRL1         0x07
#define LSM6DSR_REG_FIFO_CTRL2         0x08
#define LSM6DSR_REG_FIFO_CTRL3         0x09
#define LSM6DSR_REG_FIFO_CTRL4         0x0A
#define LSM6DSR_REG_COUNTER_BDR_REG1   0x0B
#define LSM6DSR_REG_COUNTER_BDR_REG2   0x0C
#define LSM6DSR_REG_INT1_CTRL          0x0D
#define LSM6DSR_REG_INT2_CTRL          0x0E
#define LSM6DSR_REG_WHO_AM_I           0x0F  // 固定返回值: 0x6B
#define LSM6DSR_REG_CTRL1_XL           0x10  // 加速度计控制: ODR, Full Scale
#define LSM6DSR_REG_CTRL2_G            0x11  // 陀螺仪控制: ODR, Full Scale
#define LSM6DSR_REG_CTRL3_C            0x12  // 控制3: BDU, IF_INC, SW_RESET
#define LSM6DSR_REG_CTRL4_C            0x13
#define LSM6DSR_REG_CTRL5_C            0x14
#define LSM6DSR_REG_CTRL6_C            0x15
#define LSM6DSR_REG_CTRL7_G            0x16
#define LSM6DSR_REG_CTRL8_XL           0x17
#define LSM6DSR_REG_CTRL9_XL           0x18
#define LSM6DSR_REG_CTRL10_C           0x19
#define LSM6DSR_REG_STATUS_REG         0x1E  // 数据状态寄存器
#define LSM6DSR_REG_OUT_TEMP_L         0x20
#define LSM6DSR_REG_OUT_TEMP_H         0x21

/* 陀螺仪输出寄存器 (小端对齐: L先H后, 连续6字节) */
#define LSM6DSR_REG_OUTX_L_G           0x22
#define LSM6DSR_REG_OUTX_H_G           0x23
#define LSM6DSR_REG_OUTY_L_G           0x24
#define LSM6DSR_REG_OUTY_H_G           0x25
#define LSM6DSR_REG_OUTZ_L_G           0x26
#define LSM6DSR_REG_OUTZ_H_G           0x27

/* 加速度计输出寄存器 (小端对齐: L先H后, 连续6字节) */
#define LSM6DSR_REG_OUTX_L_A           0x28
#define LSM6DSR_REG_OUTX_H_A           0x29
#define LSM6DSR_REG_OUTY_L_A           0x2A
#define LSM6DSR_REG_OUTY_H_A           0x2B
#define LSM6DSR_REG_OUTZ_L_A           0x2C
#define LSM6DSR_REG_OUTZ_H_A           0x2D

#define LSM6DSR_ID_WHO_AM_I            0x6B

/*******************************************************************************
 * 驱动导出函数
 *******************************************************************************/
uint8_t LSM6DSR_Init(void);
uint8_t LSM6DSR_Read_ID(void);
uint8_t LSM6DSR_Read_Raw(int16_t accel[3], int16_t gyro[3]);
uint8_t LSM6DSR_Get_Data(float accel_g[3], float gyro_dps[3]);

#endif /* __LSM6DSR_H__ */
