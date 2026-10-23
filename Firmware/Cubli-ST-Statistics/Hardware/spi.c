#include "spi.h"

/*******************************************************************************
 * SPI2 硬件接口初始化
 * APB1 时钟 = 42MHz
 * SPI2 波特率 = 42MHz / 8 = 5.25MHz (处于 LSM6DSRTR 10MHz 规格内)
 * Mode 3: CPOL=1, CPHA=1 (时钟空闲高电平, 第二边缘采样)
 *******************************************************************************/
void SPI2_Init(void)
{
	/* 1. 使能时钟: GPIOB 和 SPI2 */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_SPI2_CLK_ENABLE();

	/* 2. 配置 PB12 (CS) 为通用推挽输出，默认拉高 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	LSM_CS_HIGH();

	/* 3. 配置 PB13 (SCK), PB14 (MISO), PB15 (MOSI) 为复用 AF5 (SPI2) */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* 4. 配置 SPI2 寄存器 */
	SPI2->CR1 = 0; // 先清零复位
	SPI2->CR1 |= SPI_CR1_MSTR;       // 主机模式
	SPI2->CR1 |= (SPI_CR1_BR_1);     // 波特率分频: 010 -> fPCLK/8 = 5.25MHz
	SPI2->CR1 |= SPI_CR1_CPOL;       // CPOL = 1 (空闲为高电平)
	SPI2->CR1 |= SPI_CR1_CPHA;       // CPHA = 1 (第二个跳变沿采样)
	SPI2->CR1 |= SPI_CR1_SSM;        // 软件从机管理 (SSM = 1)
	SPI2->CR1 |= SPI_CR1_SSI;        // 内部 NSS 保持高电平
	/* 8位数据帧格式 (DFF = 0), MSB 先行 (LSBFIRST = 0), 全双工 (RXONLY = 0, BIDIMODE = 0) */

	/* 使能 SPI2 */
	SPI2->CR1 |= SPI_CR1_SPE;
}

/*******************************************************************************
 * SPI2 单字节收发
 *******************************************************************************/
uint8_t SPI2_ReadWriteByte(uint8_t tx_data)
{
	uint32_t timeout = 50000;
	/* 等待发送缓冲区为空 (TXE) */
	while (!(SPI2->SR & SPI_SR_TXE))
	{
		if (--timeout == 0) return 0xFF;
	}
	/* 发送数据 */
	SPI2->DR = tx_data;

	timeout = 50000;
	/* 等待接收完成 (RXNE) */
	while (!(SPI2->SR & SPI_SR_RXNE))
	{
		if (--timeout == 0) return 0xFF;
	}
	/* 返回读取的数据 */
	return (uint8_t)(SPI2->DR & 0xFF);
}

/*******************************************************************************
 * SPI 连续读数据 (寄存器读操作：Bit 7 置 1)
 *******************************************************************************/
void SPI2_ReadBytes(uint8_t reg_addr, uint8_t *rx_buf, uint16_t len)
{
	LSM_CS_LOW();
	/* 发送寄存器地址 (最高位置1表示读) */
	SPI2_ReadWriteByte(reg_addr | 0x80);
	for (uint16_t i = 0; i < len; i++)
	{
		rx_buf[i] = SPI2_ReadWriteByte(0xFF);
	}
	LSM_CS_HIGH();
}

/*******************************************************************************
 * SPI 单字节写寄存器 (寄存器写操作：Bit 7 为 0)
 *******************************************************************************/
void SPI2_WriteByte(uint8_t reg_addr, uint8_t val)
{
	LSM_CS_LOW();
	/* 发送寄存器地址 (最高位为0表示写) */
	SPI2_ReadWriteByte(reg_addr & 0x7F);
	SPI2_ReadWriteByte(val);
	LSM_CS_HIGH();
}
