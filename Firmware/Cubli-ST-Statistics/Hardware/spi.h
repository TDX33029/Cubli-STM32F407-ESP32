#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f4xx.h"
#include <stdint.h>

/* LSM6DSRTR SPI2 引脚定义:
 * PB12: LSM_CS   (软件片选输出, 低电平有效)
 * PB13: SPI2_SCK (AF5)
 * PB14: SPI2_MISO(AF5)
 * PB15: SPI2_MOSI(AF5)
 */

#define LSM_CS_LOW()   (GPIOB->BSRR = GPIO_PIN_12 << 16)
#define LSM_CS_HIGH()  (GPIOB->BSRR = GPIO_PIN_12)

void SPI2_Init(void);
uint8_t SPI2_ReadWriteByte(uint8_t tx_data);
void SPI2_ReadBytes(uint8_t reg_addr, uint8_t *rx_buf, uint16_t len);
void SPI2_WriteByte(uint8_t reg_addr, uint8_t val);

#endif /* __SPI_H__ */
