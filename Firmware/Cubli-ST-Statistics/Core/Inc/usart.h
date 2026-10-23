/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define USART_REC_LEN 256
extern unsigned char USART_RX_BUF[USART_REC_LEN];
extern volatile unsigned short USART_RX_STA;
uint8_t USART_ReadLine(char *out);
void USART_ClearCommands(void);
uint16_t USART_TxFree(void);
void UART_WriteText(const char *text);
void UART_WriteUInt(uint32_t value);
void UART_WriteInt(int32_t value);
void UART_WriteFixed(float value,uint8_t decimals);
void USART2_IRQHandler_User(void);
void USART_Link_IRQHandler(void);
void usart2_send_str(const char *str);
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

