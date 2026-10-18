#include "mock_quad_hw.h"
TIM_TypeDef quad_tim[5];
uint32_t enabled_clocks;
float voltage_power_supply=12,voltage_limit=5,velocity_limit=60;
int pole_pairs=7;
uint8_t motor_pole_pairs[3]={7,7,7};
static GPIO_InitTypeDef gpio_cfg[4];
static void *gpio_port[4];
static int gpio_calls;
static int enabled_irq=-1,disabled_irq=-1,pending_cleared=-1;
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *cfg) {assert(gpio_calls<4);gpio_port[gpio_calls]=port;gpio_cfg[gpio_calls++]=*cfg;}
void HAL_NVIC_SetPriority(int irq,int priority,int sub) {(void)irq;(void)priority;(void)sub;}
void HAL_NVIC_EnableIRQ(int irq) {enabled_irq=irq;}
void HAL_NVIC_DisableIRQ(int irq) {disabled_irq=irq;}
void HAL_NVIC_ClearPendingIRQ(int irq) {pending_cleared=irq;}
uint32_t HAL_GetTick(void) {return 0;}
uint8_t i2c_mt6701_1_get_angle(int16_t *raw,float *deg) {*raw=8192;*deg=180;return 0;}
uint8_t i2c_mt6701_2_get_angle(int16_t *raw,float *deg) {*raw=4096;*deg=90;return 0;}
uint8_t i2c_mt6701_3_get_angle(int16_t *raw,float *deg) {*raw=0;*deg=0;return 1;}
#include "quad_under_test.c"
#include "clock_under_test.c"
#include "quad_utils_under_test.c"
#include "quad_foc_under_test.c"
#if ENC_USE_HW_QUAD
/* Model an already accepted quadrature edge; electrical A/B waveforms require a real board. */
static void edge(int axis,int direction) {
    TIM_TypeDef *t=&quad_tim[axis];
    assert(t->PSC==0 && (t->SMCR&TIM_SMCR_SMS)==3);
    if(direction>0) t->CNT=t->CNT==t->ARR?0:t->CNT+1;
    else t->CNT=t->CNT==0?t->ARR:t->CNT-1;
}
static void elapsed(uint32_t us) {
    while(us--) {
        if(TIM7->CNT==0xffff) {TIM7->CNT=0;TIM7->SR|=TIM_SR_UIF;TIM7_IRQHandler();}
        else TIM7->CNT++;
    }
}
#endif
int main(void) {
    int16_t raw=-1;float deg=-1;uint32_t before;
#if ENC_USE_HW_QUAD
    int i;
#endif
    assert(ENC_Quad_GetAngle(0,&raw,&deg)!=0);
    memset(quad_tim,0xa5,sizeof(quad_tim));
    TIM5->PSC=83;TIM5->CNT=1234;
    TIM5_Micros_Init();
#if ENC_USE_HW_QUAD
    assert((enabled_clocks&8U) && !(enabled_clocks&2U));
    assert(TIM5->PSC==83 && TIM5->CNT==1234); /* timebase initialization never touches M2 */
    assert(TIM7->PSC==83 && TIM7->ARR==65535 && enabled_irq==TIM7_IRQn);
    assert(disabled_irq==TIM7_IRQn && pending_cleared==TIM7_IRQn && g_micros_hi16==0);
    TIM7->CNT=1000;before=micros();
    ENC_Quad_Init();assert(micros()==before);
    assert(gpio_calls==4);
    assert(gpio_port[0]==GPIOA && gpio_cfg[0].Pin==GPIO_PIN_5 && gpio_cfg[0].Alternate==1);
    assert(gpio_port[1]==GPIOB && gpio_cfg[1].Pin==GPIO_PIN_3 && gpio_cfg[1].Alternate==1);
    assert(gpio_port[2]==GPIOA && gpio_cfg[2].Pin==(GPIO_PIN_0|GPIO_PIN_1) && gpio_cfg[2].Alternate==2);
    assert(gpio_port[3]==GPIOC && gpio_cfg[3].Pin==(GPIO_PIN_6|GPIO_PIN_7) && gpio_cfg[3].Alternate==3);
    assert(TIM8->RCR==0);
    for(i=0;i<3;i++) {
        uint32_t count,cpr=ENC_Quad_CountsPerRev(i);
        TIM_TypeDef *t=&quad_tim[i];
        assert(t->ARR==cpr-1 && t->PSC==0 && t->DIER==0 && t->SR==0);
        assert(ENC_CountsPerRev(i)==cpr && ENC_GetAngle(i,&raw,&deg)==0 && raw==0);
        edge(i,-1);assert(ENC_GetAngle(i,&raw,&deg)==0 && raw==cpr-1);
        assert(fabsf(deg-(360.0f-360.0f/cpr))<.0001f);
        edge(i,1);assert(t->CNT==0);
        elapsed(1000);SimpleFOC_PID_Init();updateSensor(i);
        for(count=0;count<cpr;count++) {edge(i,1);elapsed(1000);updateSensor(i);}
        assert(t->CNT==0 && fabsf(shaft_angle[i]-_2PI)<.001f);
        assert(fabsf(shaft_velocity[i]-_2PI/(cpr*.001f))<.01f);
        for(count=0;count<cpr;count++) {edge(i,-1);elapsed(1000);updateSensor(i);}
        assert(t->CNT==0 && fabsf(shaft_angle[i])<.001f && shaft_velocity[i]<0);
        sensor_direction[i]=-1;elapsed(1000);edge(i,1);updateSensor(i);
        /* Converted electrical angle changes with encoder polarity, raw hardware count does not. */
        assert(fabsf(electricalAngle(i)-(_2PI-_2PI*7/cpr))<.001f);
        t->PSC=1;assert(ENC_GetAngle(i,&raw,&deg)!=0);t->PSC=0;
        t->ARR=cpr;assert(ENC_GetAngle(i,&raw,&deg)!=0);t->ARR=cpr-1;
        t->CCER|=TIM_CCER_CC1P;assert(ENC_GetAngle(i,&raw,&deg)!=0);t->CCER&=~TIM_CCER_CC1P;
        t->CR1=0;assert(ENC_GetAngle(i,&raw,&deg)!=0);t->CR1=TIM_CR1_CEN;
        t->CNT=cpr;assert(ENC_GetAngle(i,&raw,&deg)!=0);t->CNT=0;
    }
    /* Conversion must use the mapped sensor's CPR, not the motor's array index. */
    motor_sensor_map[0]=1;angle_prev_ts[0]=0;
    TIM5->CNT=ENC_Quad_CountsPerRev(1)/4;
    elapsed(1000);updateSensor(0);assert(fabsf(angle_prev[0]-_PI_2)<.0001f);
    motor_sensor_map[0]=0;
    assert(ENC_GetAngle(3,&raw,&deg)!=0 && ENC_GetAngle(0,0,&deg)!=0 && ENC_GetAngle(0,&raw,0)!=0);
    /* Pending TIM7 overflow, handled overflow and uint32 wrap all give one consistent time. */
    g_micros_hi16=2;TIM7->CNT=65535;TIM7->SR=0;before=micros();
    TIM7->CNT=0;TIM7->SR=TIM_SR_UIF;assert(micros()==before+1);TIM7_IRQHandler();assert(micros()==before+1);
    g_micros_hi16=65535;TIM7->CNT=65535;TIM7->SR=0;assert(micros()==UINT32_MAX);
    TIM7->CNT=0;TIM7->SR=TIM_SR_UIF;assert(micros()==0);TIM7_IRQHandler();assert(micros()==0);
    TIM5_Micros_Init();assert(g_micros_hi16==0 && TIM7->CNT==0 && TIM7->SR==0);
    puts("PASS: QUAD GPIO/register setup, per-axis CPR, forward/reverse/wrap, FOC scaling, invalid registers and TIM7 overflow");
#else
    assert(TIM5->PSC==83 && TIM5->ARR==UINT32_MAX && !(enabled_clocks&8U));
    before=TIM5->PSC;ENC_Quad_Init();assert(TIM5->PSC==before && !gpio_calls);
    assert(ENC_GetAngle(0,&raw,&deg)==0 && raw==8192 && deg==180 && ENC_CountsPerRev(0)==16384);
    assert(ENC_GetAngle(2,&raw,&deg)!=0 && ENC_GetAngle(0,0,&deg)!=0);
    puts("PASS: I2C routing preserved; QUAD init cannot overwrite TIM5 timebase");
#endif
    return 0;
}
