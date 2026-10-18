#include "enc_quad.h"
#include "i2c.h"

static const uint16_t quad_cpr[3]={ENC_QUAD_CPR_M1,ENC_QUAD_CPR_M2,ENC_QUAD_CPR_M3};
static uint8_t quad_initialized=0;
static TIM_TypeDef *quad_timer(uint8_t idx) {
    if(idx==0) return TIM2;
    if(idx==1) return TIM5;
    if(idx==2) return TIM8;
    return 0;
}
uint16_t ENC_Quad_CountsPerRev(uint8_t idx) {return idx<3?quad_cpr[idx]:0;}
uint16_t ENC_CountsPerRev(uint8_t idx) {
#if ENC_USE_HW_QUAD
    return ENC_Quad_CountsPerRev(idx);
#else
    return idx<3?16384U:0;
#endif
}
uint8_t ENC_I2C_GetAngle(uint8_t idx,int16_t *raw,float *deg) {
    if(!raw || !deg) return 1;
    if(idx==0) return i2c_mt6701_1_get_angle(raw,deg);
    if(idx==1) return i2c_mt6701_2_get_angle(raw,deg);
    if(idx==2) return i2c_mt6701_3_get_angle(raw,deg);
    return 1;
}
static void configure_counter(TIM_TypeDef *tim,uint16_t cpr) {
    tim->CR1=0; tim->CR2=0; tim->DIER=0;
    tim->SMCR=3U<<TIM_SMCR_SMS_Pos; /* both TI1/TI2 edges, x4 */
    tim->PSC=0;                     /* one count per accepted quadrature edge */
    tim->CCMR1=(1U<<TIM_CCMR1_CC1S_Pos)|(1U<<TIM_CCMR1_CC2S_Pos)
              |(3U<<TIM_CCMR1_IC1F_Pos)|(3U<<TIM_CCMR1_IC2F_Pos);
    tim->CCMR2=0;
    tim->CCER=TIM_CCER_CC1E|TIM_CCER_CC2E; /* input prescalers/polarity both normal */
    /* One mechanical revolution per hardware wrap, in BOTH directions.
     * 0xffff then CNT%CPR is wrong when CPR does not divide 65536. */
    tim->ARR=(uint32_t)cpr-1U;
    tim->EGR=TIM_EGR_UG;
    tim->CNT=0; tim->SR=0;
    tim->CR1=TIM_CR1_CEN;
}
void ENC_Quad_Init(void) {
#if ENC_USE_HW_QUAD
    GPIO_InitTypeDef gpio={0};
    quad_initialized=0;
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM5_CLK_ENABLE(); __HAL_RCC_TIM8_CLK_ENABLE();
    gpio.Mode=GPIO_MODE_AF_PP; gpio.Pull=GPIO_PULLUP; gpio.Speed=GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate=GPIO_AF1_TIM2;
    gpio.Pin=GPIO_PIN_5; HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_3; HAL_GPIO_Init(GPIOB,&gpio); /* PB3 is unavailable to SWO/JTAG */
    gpio.Alternate=GPIO_AF2_TIM5;
    gpio.Pin=GPIO_PIN_0|GPIO_PIN_1; HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Alternate=GPIO_AF3_TIM8;
    gpio.Pin=GPIO_PIN_6|GPIO_PIN_7; HAL_GPIO_Init(GPIOC,&gpio);
    configure_counter(TIM2,quad_cpr[0]);
    configure_counter(TIM5,quad_cpr[1]);
    TIM8->RCR=0;
    configure_counter(TIM8,quad_cpr[2]);
    quad_initialized=1;
#else
    /* Do not accidentally overwrite the TIM5 microsecond clock in I2C builds. */
    (void)configure_counter;
#endif
}
uint8_t ENC_Quad_GetAngle(uint8_t idx,int16_t *raw,float *deg) {
    TIM_TypeDef *tim=quad_timer(idx);
    uint32_t count,ccmr_mask=TIM_CCMR1_CC1S|TIM_CCMR1_CC2S|TIM_CCMR1_IC1PSC|TIM_CCMR1_IC2PSC;
    if(!tim || !raw || !deg || !quad_initialized) return 1;
    if(!(tim->CR1&TIM_CR1_CEN) || tim->PSC!=0 || tim->ARR!=quad_cpr[idx]-1U ||
       (tim->SMCR&TIM_SMCR_SMS)!=(3U<<TIM_SMCR_SMS_Pos) ||
       (tim->CCMR1&ccmr_mask)!=((1U<<TIM_CCMR1_CC1S_Pos)|(1U<<TIM_CCMR1_CC2S_Pos)) ||
       (tim->CCER&(TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC1P|TIM_CCER_CC2P|TIM_CCER_CC1NP|TIM_CCER_CC2NP))!=(TIM_CCER_CC1E|TIM_CCER_CC2E)) return 1;
    count=tim->CNT;
    if(count>=quad_cpr[idx]) return 1;
    *raw=(int16_t)count;
    *deg=(float)count*360.0f/(float)quad_cpr[idx];
    return 0;
}
uint8_t ENC_GetAngle(uint8_t idx,int16_t *raw,float *deg) {
#if ENC_USE_HW_QUAD
    return ENC_Quad_GetAngle(idx,raw,deg);
#else
    return ENC_I2C_GetAngle(idx,raw,deg);
#endif
}
