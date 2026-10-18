/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

unsigned char USART_RX_BUF[USART_REC_LEN];
volatile unsigned short USART_RX_STA = 0;
#define RX_QUEUE_SIZE 16
static char rx_queue[RX_QUEUE_SIZE][USART_REC_LEN];
static volatile uint8_t rx_head=0,rx_tail=0;
static uint16_t rx_len=0;
static uint8_t rx_overflow=0;
extern void Control_Heartbeat(void);
extern void Control_RequestStop(uint8_t reason);

/* ==========================================================================
 * 调试链路串口选择 (printf 遥测与命令解析共用的物理通道)
 *   1 = USART1 (PA9=TX, PA10=RX) -> 可选 ESP32 无线上位机链路
 *       (ESP32 IO26<-F407 PA9(TX), ESP32 IO25->F407 PA10(RX), 115200-8-N-1)
 *   2 = USART2 (PD5=TX, PD6=RX)  -> 旧板 USB-TTL 直连 PC
 * ========================================================================== */
#define DEBUG_LINK_USART   2

#if DEBUG_LINK_USART == 1
  #define DBG_LINK_UART    USART1
#else
  #define DBG_LINK_UART    USART2
#endif

/* 非阻塞高速环形发送缓冲区 (彻底杜绝 printf 阻塞 15ms 造成 FOC 控制环顿挫脉冲) */
#define UART_TX_BUF_SIZE 1024
static uint8_t uart_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t uart_tx_head = 0;
static volatile uint16_t uart_tx_tail = 0;

int fputc(int ch, FILE *f)
{
	uint16_t next_head = (uart_tx_head + 1) % UART_TX_BUF_SIZE;
	if(next_head == uart_tx_tail) return ch; /* never block motor commutation */
	uart_tx_buf[uart_tx_head] = (uint8_t)ch;
	uart_tx_head = next_head;
	DBG_LINK_UART->CR1 |= USART_CR1_TXEIE; // 使能发送中断，异步后台发送
	return ch;
}
/* Bounded fixed-point formatting: printf %f on Cortex-M4 can pause FOC for
 * >1ms even with an asynchronous UART. These helpers avoid double conversion. */
void UART_WriteText(const char *text) {while(*text) fputc((unsigned char)*text++,stdout);}
void UART_WriteUInt(uint32_t value) {
    char digits[10];int n=0;
    do {digits[n++]=(char)('0'+value%10U);value/=10U;} while(value);
    while(n) fputc(digits[--n],stdout);
}
void UART_WriteInt(int32_t value) {
    uint32_t magnitude;
    if(value<0) {fputc('-',stdout);magnitude=(uint32_t)(-(value+1))+1U;}
    else magnitude=(uint32_t)value;
    UART_WriteUInt(magnitude);
}
void UART_WriteFixed(float value,uint8_t decimals) {
    uint32_t scale=1,mag,fraction,divisor;int32_t fixed;uint8_t i;
    if(!isfinite(value)) {UART_WriteText("nan");return;}
    /* Tiny slow-loop gains need six places; retain bounded integer conversion. */
    if(decimals>6) decimals=6;
    for(i=0;i<decimals;i++) scale*=10U;
    value=value*(float)scale+(value<0?-0.5f:0.5f);
    if(value>2147483520.0f) value=2147483520.0f;
    if(value< -2147483520.0f) value=-2147483520.0f;
    fixed=(int32_t)value;
    if(fixed<0) {fputc('-',stdout);mag=(uint32_t)(-(fixed+1))+1U;} else mag=(uint32_t)fixed;
    UART_WriteUInt(mag/scale);
    if(decimals) {
        fputc('.',stdout);fraction=mag%scale;
        for(divisor=scale/10U;divisor;divisor/=10U) {fputc((int)('0'+fraction/divisor),stdout);fraction%=divisor;}
    }
}
/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
#if DEBUG_LINK_USART == 2
  USART2->CR1 |= USART_CR1_RXNEIE;   /* 仅当 USART2 被选为调试链路时才使能接收中断 */
#endif
  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN USART1_Custom */
/* USART1 init function: 板载 ESP32-WROOM32E 无线上位机链路 (PA9=TX->ESP32 IO26, PA10=RX<-ESP32 IO25) */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
#if DEBUG_LINK_USART == 1
  USART1->CR1 |= USART_CR1_RXNEIE;   /* USART1 作为调试链路时使能接收中断 */
#endif
}
/* USER CODE END USART1_Custom */

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  /* USER CODE BEGIN USART1_MspInit */
  if(uartHandle->Instance==USART1)
  {
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX (接 ESP32 IO26, ESP32 侧为 RX)
    PA10    ------> USART1_RX (接 ESP32 IO25, ESP32 侧为 TX)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  /* USER CODE END USART1_MspInit */
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void usart2_send_str(const char *str)
{
	while(str && *str)
	{
		while(!(USART2->SR & USART_SR_TXE));
		USART2->DR = (uint8_t)(*str++ & 0xFF);
	}
}

/* 调试链路公共收发中断处理 (作用于 DEBUG_LINK_UART 选定的 USART1 或 USART2)
 * 1. 发送空中断: 异步高效消费环形发送缓冲区，零等待
 * 2. 接收中断: \r\n 结尾的 ASCII 命令行组包 -> USART_RX_BUF / USART_RX_STA
 */
void USART_Link_IRQHandler(void)
{
	unsigned char Res;

	// 1. 发送空中断 (异步消费环形发送缓冲区)
	if((DBG_LINK_UART->CR1 & USART_CR1_TXEIE) && (DBG_LINK_UART->SR & USART_SR_TXE))
	{
		if(uart_tx_head != uart_tx_tail)
		{
			DBG_LINK_UART->DR = uart_tx_buf[uart_tx_tail];
			uart_tx_tail = (uart_tx_tail + 1) % UART_TX_BUF_SIZE;
		}
		else
		{
			DBG_LINK_UART->CR1 &= ~USART_CR1_TXEIE; // 发送完毕关闭发送中断
		}
	}

    /* Read SR then DR also clears ORE/FE/NE. Discard the entire damaged line. */
    {
        uint32_t sr=DBG_LINK_UART->SR;
        if(sr & (USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE)) {
            Res=(unsigned char)DBG_LINK_UART->DR;
            if(sr & (USART_SR_ORE|USART_SR_FE|USART_SR_NE)) rx_overflow=1;
            if(Res=='\n') {
                USART_RX_BUF[rx_len]=0;
                if(!rx_overflow && rx_len) {
                    char *p=(char *)USART_RX_BUF;
                    uint8_t next=(rx_head+1)%RX_QUEUE_SIZE;
                    uint16_t j=rx_len;
                    while(j && (USART_RX_BUF[j-1]==' ' || USART_RX_BUF[j-1]=='\t')) USART_RX_BUF[--j]=0;
                    for(j=0;USART_RX_BUF[j];j++) USART_RX_BUF[j]=(unsigned char)toupper(USART_RX_BUF[j]);
                    while(*p==' ' || *p=='\t') p++;
                    if(!strcmp(p,"PING") || !strcmp(p,"ping")) Control_Heartbeat();
                    else if(!strcmp(p,"STOP") || !strcmp(p,"stop") || !strcmp(p,"S") || !strcmp(p,"s")) {
                        rx_tail=rx_head; Control_RequestStop(1);
                    } else if(next==rx_tail) {Control_RequestStop(4);rx_tail=rx_head;}
                    else {strcpy(rx_queue[rx_head],p);rx_head=next;}
                }
                rx_len=0;rx_overflow=0;
            } else if(Res!='\r') {
                if(rx_len<USART_REC_LEN-1) USART_RX_BUF[rx_len++]=Res;
                else rx_overflow=1;
            }
        }
    }
}
uint16_t USART_TxFree(void) {
    return (uart_tx_tail+UART_TX_BUF_SIZE-uart_tx_head-1)%UART_TX_BUF_SIZE;
}
uint8_t USART_ReadLine(char *out) {
    uint32_t irq=__get_PRIMASK();
    __disable_irq();
    if(rx_head==rx_tail) {__set_PRIMASK(irq);return 0;}
    strcpy(out,rx_queue[rx_tail]);rx_tail=(rx_tail+1)%RX_QUEUE_SIZE;
    __set_PRIMASK(irq);return 1;
}
void USART_ClearCommands(void) {
    uint32_t irq=__get_PRIMASK(); __disable_irq();rx_tail=rx_head;__set_PRIMASK(irq);
}

/* 兼容旧中断文件命名 (stm32f4xx_it.c 的 USART2_IRQHandler 调用) */
void USART2_IRQHandler_User(void)
{
	USART_Link_IRQHandler();
}
/* USER CODE END 1 */

