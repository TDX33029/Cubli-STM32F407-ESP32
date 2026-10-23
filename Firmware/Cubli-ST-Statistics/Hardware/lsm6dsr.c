#include "lsm6dsr.h"
#include "spi.h"
#include "delay.h"

/*******************************************************************************
 * 读取 LSM6DSRTR WHO_AM_I ID (应返回 0x6B)
 *******************************************************************************/
uint8_t LSM6DSR_Read_ID(void)
{
	uint8_t id = 0;
	SPI2_ReadBytes(LSM6DSR_REG_WHO_AM_I, &id, 1);
	return id;
}

/*******************************************************************************
 * 初始化 LSM6DSRTR
 * 1. 软件复位
 * 2. 检查 WHO_AM_I ID (0x6B)
 * 3. 启用 BDU (块数据更新) 与 IF_INC (连续读地址自增)
 * 4. 配置加速度计: 416Hz, ±2g (CTRL1_XL = 0x60)
 * 5. 配置陀螺仪: 416Hz, ±2000dps (CTRL2_G = 0x6C)
 * 返回值: 0=成功, 1=设备ID错误/无响应
 *******************************************************************************/
uint8_t LSM6DSR_Init(void)
{
	SPI2_Init();
	delay_ms(20);

	/* 1. 软件复位 */
	SPI2_WriteByte(LSM6DSR_REG_CTRL3_C, 0x01); // SW_RESET
	delay_ms(50);

	/* 2. 读取器件ID并校验 */
	uint8_t who = LSM6DSR_Read_ID();
	if (who != LSM6DSR_ID_WHO_AM_I)
	{
		return 1; // 校验失败
	}

	/* 3. CTRL3_C: BDU=1 (Bit 6), IF_INC=1 (Bit 2) -> 0x44 */
	SPI2_WriteByte(LSM6DSR_REG_CTRL3_C, 0x44);

	/* 4. 加速度计 CTRL1_XL:
	 * ODR_XL = 0110 (416 Hz)
	 * FS_XL  = 00 (±2g, 灵敏度 0.061 mg/LSB)
	 * LPF2_XL_EN = 0
	 * 写入 0x60
	 */
	SPI2_WriteByte(LSM6DSR_REG_CTRL1_XL, 0x60);

	/* 5. 陀螺仪 CTRL2_G:
	 * ODR_G = 0110 (416 Hz)
	 * FS_G  = 1100 (±2000 dps, 灵敏度 70 mdps/LSB)
	 * 写入 0x6C
	 */
	SPI2_WriteByte(LSM6DSR_REG_CTRL2_G, 0x6C);

	delay_ms(10);
	return 0;
}

/*******************************************************************************
 * 突发读取 6 轴原始数据 (12字节连续地址: 0x22 ~ 0x2D)
 * 耗时约 25us，对电机开环/闭环执行循环完全不造成任何可感知的延迟
 *******************************************************************************/
uint8_t LSM6DSR_Read_Raw(int16_t accel[3], int16_t gyro[3])
{
	uint8_t raw[12];
    if(LSM6DSR_Read_ID()!=LSM6DSR_ID_WHO_AM_I) return 1;
	/* 从 0x22 (OUTX_L_G) 连续突发读取 12 字节 */
	SPI2_ReadBytes(LSM6DSR_REG_OUTX_L_G, raw, 12);

	/* 陀螺仪: 小端对齐 (L在前, H在后) */
	gyro[0] = (int16_t)((raw[1] << 8) | raw[0]); // Gx
	gyro[1] = (int16_t)((raw[3] << 8) | raw[2]); // Gy
	gyro[2] = (int16_t)((raw[5] << 8) | raw[4]); // Gz

	/* 加速度计: 小端对齐 */
	accel[0] = (int16_t)((raw[7] << 8) | raw[6]); // Ax
	accel[1] = (int16_t)((raw[9] << 8) | raw[8]); // Ay
	accel[2] = (int16_t)((raw[11] << 8) | raw[10]); // Az

	return 0;
}

/*******************************************************************************
 * 获取转换后的物理量
 * accel_g: 单位为 g (重力加速度, 1g ≈ 9.8m/s²)
 * gyro_dps: 单位为 °/s (度每秒)
 *
 * 比例常数 (参考官方规格书):
 * ±2g 灵敏度: 0.061 mg/LSB = 0.000061 g/LSB
 * ±2000dps 灵敏度: 70 mdps/LSB = 0.070 dps/LSB
 *******************************************************************************/
uint8_t LSM6DSR_Get_Data(float accel_g[3], float gyro_dps[3])
{
	int16_t raw_a[3] = {0};
	int16_t raw_g[3] = {0};

	if(LSM6DSR_Read_Raw(raw_a, raw_g)!=0) return 1;

	/* 转换加速度 (g) */
	accel_g[0] = (float)raw_a[0] * 0.000061f;
	accel_g[1] = (float)raw_a[1] * 0.000061f;
	accel_g[2] = (float)raw_a[2] * 0.000061f;

	/* 转换角速度 (dps) */
	gyro_dps[0] = (float)raw_g[0] * 0.070f;
	gyro_dps[1] = (float)raw_g[1] * 0.070f;
	gyro_dps[2] = (float)raw_g[2] * 0.070f;
    return 0;
}
