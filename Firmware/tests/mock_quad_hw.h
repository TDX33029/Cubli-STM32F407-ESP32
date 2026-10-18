#ifndef MOCK_QUAD_HW_H
#define MOCK_QUAD_HW_H
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
typedef struct {uint32_t CR1,CR2,SMCR,DIER,SR,EGR,CCMR1,CCMR2,CCER,CNT,PSC,ARR,RCR;} TIM_TypeDef;
extern TIM_TypeDef quad_tim[5];
#define TIM2 (&quad_tim[0])
#define TIM5 (&quad_tim[1])
#define TIM8 (&quad_tim[2])
#define TIM7 (&quad_tim[3])
#define TIM6 (&quad_tim[4])
#define TIM_CR1_CEN 1U
#define TIM_SR_UIF 1U
#define TIM_DIER_UIE 1U
#define TIM_EGR_UG 1U
#define TIM_SMCR_SMS_Pos 0U
#define TIM_SMCR_SMS 7U
#define TIM_CCMR1_CC1S_Pos 0U
#define TIM_CCMR1_CC2S_Pos 8U
#define TIM_CCMR1_IC1F_Pos 4U
#define TIM_CCMR1_IC2F_Pos 12U
#define TIM_CCMR1_CC1S 3U
#define TIM_CCMR1_CC2S (3U<<8)
#define TIM_CCMR1_IC1PSC (3U<<2)
#define TIM_CCMR1_IC2PSC (3U<<10)
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC1P 2U
#define TIM_CCER_CC1NP 8U
#define TIM_CCER_CC2E 16U
#define TIM_CCER_CC2P 32U
#define TIM_CCER_CC2NP 128U
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_3 8U
#define GPIO_PIN_5 32U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIOA ((void *)(uintptr_t)1)
#define GPIOB ((void *)(uintptr_t)2)
#define GPIOC ((void *)(uintptr_t)3)
#define GPIO_MODE_AF_PP 2U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_HIGH 2U
#define GPIO_AF1_TIM2 1U
#define GPIO_AF2_TIM5 2U
#define GPIO_AF3_TIM8 3U
#define TIM7_IRQn 55
#define TIM6_DAC_IRQn 54
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
extern uint32_t enabled_clocks;
#define __HAL_RCC_TIM2_CLK_ENABLE() (enabled_clocks|=1U)
#define __HAL_RCC_TIM5_CLK_ENABLE() (enabled_clocks|=2U)
#define __HAL_RCC_TIM8_CLK_ENABLE() (enabled_clocks|=4U)
#define __HAL_RCC_TIM7_CLK_ENABLE() (enabled_clocks|=8U)
#define __HAL_RCC_TIM6_CLK_ENABLE() (enabled_clocks|=16U)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *cfg);
void HAL_NVIC_SetPriority(int irq,int priority,int sub);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
uint32_t HAL_GetTick(void);
uint8_t i2c_mt6701_1_get_angle(int16_t *raw,float *deg);
uint8_t i2c_mt6701_2_get_angle(int16_t *raw,float *deg);
uint8_t i2c_mt6701_3_get_angle(int16_t *raw,float *deg);
#include "enc_quad.h"
#include "timer.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#endif
