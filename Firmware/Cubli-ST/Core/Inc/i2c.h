/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#define MT6701_SLAVE_ADDR		  (0x06 << 1)
#define MT6701_Timeout            50

#define MT6701_REG_ANGLE_14b      0x03

#define mt6701_log		printf

void i2c_bus_recover(I2C_HandleTypeDef *hi2c);
uint8_t i2c_mt6701_1_get_angle(int16_t *angle, float *angle_f);
uint8_t i2c_mt6701_2_get_angle(int16_t *angle, float *angle_f);
uint8_t i2c_mt6701_3_get_angle(int16_t *angle, float *angle_f);

extern uint32_t i2c1_last_err;
extern uint8_t i2c1_last_hal_res;

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

extern I2C_HandleTypeDef hi2c2;

extern I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_I2C3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

