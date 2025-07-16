/*
 * 	File		: clock.h
 *	Created on	: Jul 12, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */

#ifndef INC_CLOCK_H_
#define INC_CLOCK_H_

typedef enum
{
	APB2_TIM1 = 0,
	APB2_USART1 = 4,
	APB2_USART6 = 5,
	APB2_ADC1 = 8,
	APB2_SDIO = 11,
	APB2_SPI1 = 12,
	APB2_SPI4 = 13,
	APB2_SYSCFG = 14,
	APB2_TIM9 = 16,
	APB2_TIM10 = 17,
	APB2_TIM11 = 18,
	APB2_SPI5 = 20,
} APB2_periph_t;

typedef enum
{
	APB1_TIM2 = 0,
	APB1_TIM3 = 1,
	APB1_TIM4 = 2,
	APB1_TIM5 = 3,
	APB1_WWDG = 11,
	APB1_SPI2 = 14,
	APB1_SPI3 = 15,
	APB1_USART2 = 17,
	APB1_I2C1 = 21,
	APB1_I2C2 = 22,
	APB1_I2C3 = 23,
	APB1_PWR  = 28
} APB1_periph_t;

typedef enum
{
	AHB2_OTGFS = 7
} AHB2_periph_t;

typedef enum
{
	AHB1_GPIOA = 0,
	AHB1_GPIOB = 1,
	AHB1_GPIOC = 2,
	AHB1_GPIOD = 3,
	AHB1_GPIOE = 4,
	AHB1_GPIOH = 7,
	AHB1_CRC   = 12,
	AHB1_DMA1  = 21,
	AHB1_DMA2  = 22
} AHB1_periph_t;

#define RCC_BASE_ADDR 0x40023800
#define FLASH_INTF_BASE_ADDR 0x40023C00
void APB2_clock_enable(APB2_periph_t peripheral);
void APB1_clock_enable(APB1_periph_t peripheral);
void AHB2_clock_enable(AHB2_periph_t peripheral);
void AHB1_clock_enable(AHB1_periph_t peripheral);
void RCC_Init();

#endif /* INC_CLOCK_H_ */
