
#include "delay.h"
#include "timer.h"

/******************************************************************************/
void DWT_Init(void)
{
	// 保留函数兼容，系统已全面切换至 TIM5 32位全硬件微秒定时器
}

// 延时nus (基于 TIM5 32位 1MHz 全硬件计数器，绝对可靠)
void delay_us(unsigned long nus)
{
	uint32_t start = micros();
	while((uint32_t)(micros() - start) < (uint32_t)nus);
}

// 延时nms (基于 TIM6 1ms 中断驱动的 HAL_GetTick()，无上限且不受任务阻塞影响)
void delay_ms(unsigned short nms)
{
	uint32_t start = HAL_GetTick();
	while((uint32_t)(HAL_GetTick() - start) < (uint32_t)nms);
}

// 兼容空函数
void systick_CountMode(void)
{
}
/******************************************************************************/
