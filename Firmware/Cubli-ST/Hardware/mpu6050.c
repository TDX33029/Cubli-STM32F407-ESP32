#include "mpu6050.h"
#include "stm32f4xx.h"
#include "delay.h"

/*******************************************************************************
 * PB10 (SCL), PB11 (SDA) 软件模拟 I2C 实现
 * 开漏上拉模式，直接操作引脚即可读取和拉低
 *******************************************************************************/
#define SCL_H()   (GPIOB->BSRR = GPIO_PIN_10)
#define SCL_L()   (GPIOB->BSRR = (GPIO_PIN_10 << 16))
#define SDA_H()   (GPIOB->BSRR = GPIO_PIN_11)
#define SDA_L()   (GPIOB->BSRR = (GPIO_PIN_11 << 16))
#define SDA_READ() ((GPIOB->IDR & GPIO_PIN_11) ? 1 : 0)

static void i2c_delay(void)
{
	for (volatile int i = 0; i < 25; i++);
}

static void i2c_start(void)
{
	SDA_H();
	SCL_H();
	i2c_delay();
	SDA_L();
	i2c_delay();
	SCL_L();
	i2c_delay();
}

static void i2c_stop(void)
{
	SDA_L();
	SCL_H();
	i2c_delay();
	SDA_H();
	i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t dat)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (dat & 0x80) SDA_H(); else SDA_L();
		dat <<= 1;
		i2c_delay();
		SCL_H();
		i2c_delay();
		SCL_L();
		i2c_delay();
	}
	SDA_H(); // 释放SDA
	i2c_delay();
	SCL_H();
	i2c_delay();
	uint8_t ack = SDA_READ();
	SCL_L();
	i2c_delay();
	return ack == 0 ? 0 : 1; // 0=ACK, 1=NACK
}

static uint8_t i2c_read_byte(uint8_t ack)
{
	uint8_t dat = 0;
	SDA_H();
	for (uint8_t i = 0; i < 8; i++)
	{
		dat <<= 1;
		SCL_H();
		i2c_delay();
		if (SDA_READ()) dat |= 0x01;
		SCL_L();
		i2c_delay();
	}
	if (ack) SDA_L(); else SDA_H();
	i2c_delay();
	SCL_H();
	i2c_delay();
	SCL_L();
	SDA_H();
	i2c_delay();
	return dat;
}

static uint8_t mpu_write_reg(uint8_t reg, uint8_t val)
{
	i2c_start();
	if (i2c_write_byte((MPU6050_ADDR << 1) | 0)) { i2c_stop(); return 1; }
	if (i2c_write_byte(reg)) { i2c_stop(); return 1; }
	if (i2c_write_byte(val)) { i2c_stop(); return 1; }
	i2c_stop();
	return 0;
}

static uint8_t mpu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
	i2c_start();
	if (i2c_write_byte((MPU6050_ADDR << 1) | 0)) { i2c_stop(); return 1; }
	if (i2c_write_byte(reg)) { i2c_stop(); return 1; }
	i2c_start();
	if (i2c_write_byte((MPU6050_ADDR << 1) | 1)) { i2c_stop(); return 1; }
	for (uint8_t i = 0; i < len; i++)
	{
		buf[i] = i2c_read_byte(i == (len - 1) ? 0 : 1);
	}
	i2c_stop();
	return 0;
}

/*******************************************************************************
 * MPU-6050 初始化
 *******************************************************************************/
uint8_t MPU6050_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 开漏输出
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	SCL_H();
	SDA_H();
	delay_ms(20);

	/* 复位并唤醒 MPU6050 */
	mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80); // 复位
	delay_ms(50);
	mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00); // 唤醒, 内部8MHz
	delay_ms(20);

	uint8_t id = 0;
	mpu_read_regs(MPU_REG_WHO_AM_I, &id, 1);
	if (id != 0x68)
	{
		return 1;
	}

	mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x04);   // 200Hz 采样率
	mpu_write_reg(MPU_REG_CONFIG, 0x02);       // 加速度94Hz/陀螺仪98Hz低通，避免平衡控制大相位延迟
	mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x18);  // ±2000 dps
	mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00); // ±2g
	return 0;
}

uint8_t MPU6050_Read_ID(void)
{
	uint8_t id = 0;
	mpu_read_regs(MPU_REG_WHO_AM_I, &id, 1);
	return id;
}

uint8_t MPU6050_Read_Raw(int16_t accel[3], int16_t gyro[3])
{
	uint8_t buf[14];
	if (mpu_read_regs(MPU_REG_ACCEL_XOUT_H, buf, 14) != 0)
	{
		return 1;
	}
	/* MPU-6050 大端对齐 (H先L后) */
	accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
	accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
	accel[2] = (int16_t)((buf[4] << 8) | buf[5]);

	gyro[0] = (int16_t)((buf[8] << 8) | buf[9]);
	gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
	gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);
	return 0;
}

uint8_t MPU6050_Get_Data(float accel_g[3], float gyro_dps[3])
{
	int16_t raw_a[3] = {0};
	int16_t raw_g[3] = {0};
	if(MPU6050_Read_Raw(raw_a, raw_g)!=0) return 1;
	/* ±2g -> 16384 LSB/g */
	accel_g[0] = (float)raw_a[0] / 16384.0f;
	accel_g[1] = (float)raw_a[1] / 16384.0f;
	accel_g[2] = (float)raw_a[2] / 16384.0f;

	/* ±2000 dps -> 16.4 LSB/(°/s) */
	gyro_dps[0] = (float)raw_g[0] / 16.4f;
	gyro_dps[1] = (float)raw_g[1] / 16.4f;
	gyro_dps[2] = (float)raw_g[2] / 16.4f;
    return 0;
}
