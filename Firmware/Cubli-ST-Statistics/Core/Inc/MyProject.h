#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "i2c.h"

#include "delay.h"
#include "timer.h"
#include "spi.h"
#include "lsm6dsr.h"
#include "mpu6050.h"
#include "enc_quad.h"

/* 陀螺仪传感器选择宏 (1: LSM6DSRTR 硬件SPI2默认启用; 2: MPU-6050 备用软件I2C) */
#define IMU_TYPE_LSM6DSR   1
#define IMU_TYPE_MPU6050   2
#define ACTIVE_IMU         IMU_TYPE_LSM6DSR

#include "foc_utils.h"
#include "FOCMotor.h"
#include "BLDCmotor.h"
#include "CurrentSense.h"
#include "lqr_balance.h"

/* 调试串口重定向与标准库 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/******************************************************************************/
/* DRV8313 使能引脚: PD0=M1, PD1=M2, PD2=M3 */
#define M1_Enable    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET)
#define M1_Disable   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET)
#define M2_Enable    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET)
#define M2_Disable   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET)
#define M3_Enable    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET)
#define M3_Disable   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)

/* 外部变量声明 */
extern volatile uint32_t time1_cntr;

/* 自检与状态指示功能接口 */
void Boot_PG4_Blink_5s(void);
void Motor_Encoder_OpenLoop_SelfTest(void);

#endif
