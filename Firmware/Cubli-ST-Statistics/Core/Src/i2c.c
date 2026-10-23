/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "delay.h"

/* USER CODE BEGIN 0 */
#define MT6701_READ_TIMEOUT 10  // 提升读超时为 10ms，提高噪声与突发环境下的鲁棒性

// I2C 硬件状态机与从机死锁9脉冲恢复函数 (彻底解决从机拉低SDA导致的硬件BUSY挂死)
void i2c_bus_recover(I2C_HandleTypeDef *hd_i2c)
{
	GPIO_TypeDef *scl_port, *sda_port;
	uint16_t scl_pin, sda_pin;
	uint8_t af;
	int i;

	if(hd_i2c->Instance == I2C1)
	{
		scl_port = GPIOB; scl_pin = GPIO_PIN_6;
		sda_port = GPIOB; sda_pin = GPIO_PIN_7;
		af = GPIO_AF4_I2C1;
	}
	else if(hd_i2c->Instance == I2C2)
	{
		scl_port = GPIOF; scl_pin = GPIO_PIN_1;
		sda_port = GPIOF; sda_pin = GPIO_PIN_0;
		af = GPIO_AF4_I2C2;
	}
	else if(hd_i2c->Instance == I2C3)
	{
		scl_port = GPIOA; scl_pin = GPIO_PIN_8;
		sda_port = GPIOC; sda_pin = GPIO_PIN_9;
		af = GPIO_AF4_I2C3;
	}
	else return;

	// 只要从机把SDA拉低或者硬件标志位处于BUSY，立即执行9脉冲强制时钟驱逐
	if(HAL_GPIO_ReadPin(sda_port, sda_pin) == GPIO_PIN_RESET || (hd_i2c->Instance->SR2 & I2C_SR2_BUSY))
	{
		// 1. 先复位并关闭 I2C 硬件外设
		hd_i2c->Instance->CR1 |= I2C_CR1_SWRST;
		delay_us(5);
		hd_i2c->Instance->CR1 &= ~I2C_CR1_SWRST;
		__HAL_I2C_DISABLE(hd_i2c);

		// 2. 将 SCL 与 SDA 暂时切为开漏 GPIO
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = scl_pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(scl_port, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = sda_pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(sda_port, &GPIO_InitStruct);

		// 3. 连续发出最多 9 个 SCL 脉冲，驱动从机移出位并释放 SDA
		for(i = 0; i < 9; i++)
		{
			HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
			delay_us(5);
			HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
			delay_us(5);
			if(HAL_GPIO_ReadPin(sda_port, sda_pin) == GPIO_PIN_SET)
			{
				break; // 从机已成功释放 SDA 线
			}
		}

		// 4. 产生硬件 STOP 停止信号
		GPIO_InitStruct.Pin = sda_pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		HAL_GPIO_Init(sda_port, &GPIO_InitStruct);

		HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
		delay_us(5);
		HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
		delay_us(5);
		HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
		delay_us(5);

		// 5. 恢复为 I2C 复用功能并重新初始化
		GPIO_InitStruct.Pin = scl_pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = af;
		HAL_GPIO_Init(scl_port, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = sda_pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = af;
		HAL_GPIO_Init(sda_port, &GPIO_InitStruct);

		HAL_I2C_Init(hd_i2c);
	}
}

unsigned char mt6701_write_reg(I2C_HandleTypeDef *hd_i2c, unsigned char reg, unsigned char value)
{
	HAL_StatusTypeDef res = HAL_I2C_Mem_Write(hd_i2c, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, MT6701_READ_TIMEOUT);
	if (res != HAL_OK) { i2c_bus_recover(hd_i2c); hd_i2c->State = HAL_I2C_STATE_READY; }
	return res;
}

unsigned char mt6701_write_regs(I2C_HandleTypeDef *hd_i2c, unsigned char reg, unsigned char *value, unsigned char len)
{
	HAL_StatusTypeDef res = HAL_I2C_Mem_Write(hd_i2c, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, len, MT6701_READ_TIMEOUT);
	if (res != HAL_OK) { i2c_bus_recover(hd_i2c); hd_i2c->State = HAL_I2C_STATE_READY; }
	return res;
}

unsigned char mt6701_read_reg(I2C_HandleTypeDef *hd_i2c, unsigned char reg, unsigned char* buf, unsigned short len)
{
	HAL_StatusTypeDef res = HAL_I2C_Mem_Read(hd_i2c, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, MT6701_READ_TIMEOUT);
	if (res != HAL_OK) { i2c_bus_recover(hd_i2c); hd_i2c->State = HAL_I2C_STATE_READY; }
	return res;
}

uint32_t i2c1_last_err = 0;
uint8_t i2c1_last_hal_res = 0;

uint8_t i2c_mt6701_1_get_angle(int16_t *angle, float *angle_f)
{
    uint8_t temp[2] = {0};
    HAL_StatusTypeDef res;
    if (hi2c1.Instance && (hi2c1.Instance->SR2 & I2C_SR2_BUSY))
    {
        i2c_bus_recover(&hi2c1);
    }
    res = HAL_I2C_Mem_Read(&hi2c1, MT6701_SLAVE_ADDR, MT6701_REG_ANGLE_14b, I2C_MEMADD_SIZE_8BIT, temp, 2, MT6701_READ_TIMEOUT);
    if (res == HAL_OK)
    {
        *angle = ((int16_t)temp[0] << 6) | (temp[1] >> 2);
        *angle_f = (float)*angle * 360.0f / 16384.0f;
        return 0; // 正常
    }
    i2c1_last_hal_res = (uint8_t)res;
    i2c1_last_err = hi2c1.ErrorCode;
    i2c_bus_recover(&hi2c1);
    hi2c1.State = HAL_I2C_STATE_READY;
    return 1; // 离线或错误
}

uint8_t i2c_mt6701_2_get_angle(int16_t *angle, float *angle_f)
{
    uint8_t temp[2] = {0};
    if (hi2c2.Instance && (hi2c2.Instance->SR2 & I2C_SR2_BUSY))
    {
        i2c_bus_recover(&hi2c2);
    }
    if (HAL_I2C_Mem_Read(&hi2c2, MT6701_SLAVE_ADDR, MT6701_REG_ANGLE_14b, I2C_MEMADD_SIZE_8BIT, temp, 2, MT6701_READ_TIMEOUT) == HAL_OK)
    {
        *angle = ((int16_t)temp[0] << 6) | (temp[1] >> 2);
        *angle_f = (float)*angle * 360.0f / 16384.0f;
        return 0; // 正常
    }
    i2c_bus_recover(&hi2c2);
    hi2c2.State = HAL_I2C_STATE_READY;
    return 1; // 离线或错误
}

uint8_t i2c_mt6701_3_get_angle(int16_t *angle, float *angle_f)
{
    uint8_t temp[2] = {0};
    if (hi2c3.Instance && (hi2c3.Instance->SR2 & I2C_SR2_BUSY))
    {
        i2c_bus_recover(&hi2c3);
    }
    if (HAL_I2C_Mem_Read(&hi2c3, MT6701_SLAVE_ADDR, MT6701_REG_ANGLE_14b, I2C_MEMADD_SIZE_8BIT, temp, 2, MT6701_READ_TIMEOUT) == HAL_OK)
    {
        *angle = ((int16_t)temp[0] << 6) | (temp[1] >> 2);
        *angle_f = (float)*angle * 360.0f / 16384.0f;
        return 0; // 正常
    }
    i2c_bus_recover(&hi2c3);
    hi2c3.State = HAL_I2C_STATE_READY;
    return 1; // 离线或错误
}

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}
/* I2C3 init function */
void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_1);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

