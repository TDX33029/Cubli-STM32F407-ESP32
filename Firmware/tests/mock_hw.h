#ifndef MOCK_HW_H
#define MOCK_HW_H
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <assert.h>
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_9 512U
#define GPIO_PIN_10 1024U
#define GPIO_PIN_11 2048U
#define USART_REC_LEN 256
#define ACTIVE_IMU 1
#ifndef ENC_USE_HW_QUAD
#define ENC_USE_HW_QUAD 0
#endif
#define PWM_Period_TIM1 3359
#define PWM_Period_APB1 1679
#define __get_PRIMASK() 0U
#define __set_PRIMASK(x) ((void)(x))
#define __disable_irq() ((void)0)
typedef struct {uint32_t CCR1,CCR2,CCR3;} TIM_TypeDef;
typedef struct {uint32_t IDR,BSRR;} GPIO_TypeDef;
extern TIM_TypeDef mock_timers[3];
extern GPIO_TypeDef mock_gpio;
#define TIM1 (&mock_timers[0])
#define TIM3 (&mock_timers[1])
#define TIM4 (&mock_timers[2])
#define GPIOD (&mock_gpio)
#define GPIOG (&mock_gpio)
uint32_t HAL_GetTick(void);
uint32_t micros(void);
void delay_ms(unsigned short ms);
void delay_us(unsigned long us);
void Control_RequestStop(uint8_t reason);
void Control_Heartbeat(void);
uint8_t Control_Aborted(void);
void set_motor_enable(uint8_t motor,uint8_t en);
uint8_t USART_ReadLine(char *line);
void USART_ClearCommands(void);
uint16_t USART_TxFree(void);
void UART_WriteText(const char *text);
void UART_WriteUInt(uint32_t value);
void UART_WriteInt(int32_t value);
void UART_WriteFixed(float value,uint8_t decimals);
uint16_t ENC_CountsPerRev(uint8_t idx);
uint8_t ENC_GetAngle(uint8_t idx,int16_t *raw,float *deg);
uint8_t LSM6DSR_Init(void);
uint8_t MPU6050_Init(void);
#include "foc_utils.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "lqr_balance.h"
#endif
