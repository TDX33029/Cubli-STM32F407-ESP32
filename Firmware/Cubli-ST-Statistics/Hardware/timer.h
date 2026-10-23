#ifndef STM32_TIMER_H
#define STM32_TIMER_H

/******************************************************************************/
#include "main.h"

/* PWM周期: TIM1时钟168MHz, 中心对齐模式
 * 频率 = 168MHz / (2 * (PWM_Period_TIM1 + 1))
 * PWM_Period_TIM1 = 3359 -> 25KHz
 */
#define PWM_Period_TIM1   3359
/* TIM3/TIM4时钟84MHz (APB1), 中心对齐模式
 * 频率 = 84MHz / (2 * (PWM_Period_APB1 + 1))
 * PWM_Period_APB1 = 1679 -> 25KHz
 */
#define PWM_Period_APB1   1679

/******************************************************************************/
void TIM6_1ms_Init(void);
void TIM5_Micros_Init(void);
uint32_t micros(void);
/******************************************************************************/

#endif /* STM32_TIMER_H */
