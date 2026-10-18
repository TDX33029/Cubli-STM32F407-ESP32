
#include "timer.h"
#include "enc_quad.h"

/***************************************************************************/
/* TIM1已由CubeMX的MX_TIM1_Init()初始化 (Core/Src/tim.c)
 * TIM1: PE9=CH1, PE11=CH2, PE13=CH3, 25KHz, 中心对齐, deadtime=60ns
 * 这里只保留TIM6初始化
 */

/***************************************************************************/
/* TIM6 1ms interrupt
 * TIM6 clock = APB1 timer clock = 84MHz
 * Prescaler = 84-1 -> 1MHz, Period = 1000-1 -> 1ms
 *
 * 使用寄存器级操作 (不依赖 HAL TIM 驱动)
 */
void TIM6_1ms_Init(void)
{
	__HAL_RCC_TIM6_CLK_ENABLE();

	/* Enable TIM6 interrupt in NVIC */
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	/* 关闭定时器 (配置期间) */
	TIM6->CR1 = 0;

	/* 预分频器: 84-1 -> 1MHz (84MHz / 84 = 1MHz) */
	TIM6->PSC = 84 - 1;
	/* 自动重装载: 1000-1 -> 1ms */
	TIM6->ARR = 1000 - 1;

	/* 使能更新中断 */
	TIM6->DIER = TIM_DIER_UIE;

	/* 生成更新事件 */
	TIM6->EGR |= TIM_EGR_UG;

	/* 使能定时器 */
	TIM6->CR1 |= TIM_CR1_CEN;
}

/***************************************************************************/
/* 微秒时基定时器
 *
 * 默认模式 (ENC_USE_HW_QUAD=0): 使用 TIM5 32位自由计数器, 1MHz, 约71.5分钟回绕。
 * 硬件正交解码模式 (ENC_USE_HW_QUAD=1): TIM5 让位给 M2 编码器 (PA0/PA1 只能
 * 映射到 TIM2/TIM5, 而 TIM2 被 M1 占用), 微秒时基迁移到 TIM7:
 *   TIM7: 16位, 1MHz (PSC=84-1, ARR=0xFFFF), 每65.536ms溢出一次,
 *   由更新中断维护32位高16位扩展, micros() 组合输出, uint32 回绕特性与
 *   原实现完全一致 (所有 (uint32_t)(now - last) 差值计算无需改动)。
 */
static volatile uint32_t g_micros_hi16 = 0;   /* 65536us 单元计数 (TIM7 溢出中断维护) */

void TIM7_IRQHandler(void)
{
	if (TIM7->SR & TIM_SR_UIF)
	{
		TIM7->SR = ~TIM_SR_UIF;
		g_micros_hi16++;
	}
}

void TIM5_Micros_Init(void)
{
#if ENC_USE_HW_QUAD
	__HAL_RCC_TIM7_CLK_ENABLE();

	/* TIM7 时基: 1MHz 自由计数; initialize before enabling its IRQ. */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
	TIM7->CR1 = 0;
    TIM7->DIER = 0;
    g_micros_hi16 = 0;
	TIM7->PSC = 84 - 1;
	TIM7->ARR = 0xFFFF;
	TIM7->CNT = 0;
	TIM7->EGR = TIM_EGR_UG;
    TIM7->SR = 0;
    HAL_NVIC_ClearPendingIRQ(TIM7_IRQn);
    TIM7->DIER = TIM_DIER_UIE;

	/* 溢出中断优先级设为最高, 确保 65.536ms 周期的溢出计数绝不丢失 */
	HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);

	TIM7->CR1 |= TIM_CR1_CEN;
#else
	__HAL_RCC_TIM5_CLK_ENABLE();
	TIM5->CR1 = 0;
	TIM5->PSC = 84 - 1;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->CNT = 0;
	TIM5->EGR |= TIM_EGR_UG;
	TIM5->CR1 |= TIM_CR1_CEN;
#endif
}

uint32_t micros(void)
{
#if ENC_USE_HW_QUAD
    uint32_t irq=__get_PRIMASK(),hi;
    uint16_t cnt;
    __disable_irq();
    hi=g_micros_hi16;cnt=(uint16_t)TIM7->CNT;
    /* Account for an overflow pending while a higher-priority ISR masks TIM7. */
    if(TIM7->SR & TIM_SR_UIF) {hi++;cnt=(uint16_t)TIM7->CNT;}
    __set_PRIMASK(irq);
    return (hi<<16)|cnt;
#else
	return TIM5->CNT;
#endif
}
/***************************************************************************/

/***************************************************************************/
