/*
 * 	File		: clock.c
 *	Created on	: Jul 12, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */

#include "main.h"
#include "clock.h"

/**
 * @brief  enable clock for APB2 peripheral
 * @param  peripheral: name of the APB2 peripheral you want enable clock
 */
void APB2_clock_enable(APB2_periph_t peripheral)
{
	uint32_t* RCC_APB2ENR = (uint32_t*) (RCC_BASE_ADDR + 0x44);
	*RCC_APB2ENR |= 1 << peripheral;
}

/**
 * @brief  enable clock for APB1 peripheral
 * @param  peripheral: name of the APB1 peripheral you want enable clock
 */
void APB1_clock_enable(APB1_periph_t peripheral)
{
	uint32_t* RCC_APB1ENR = (uint32_t*) (RCC_BASE_ADDR + 0x40);
	*RCC_APB1ENR |= 1 << peripheral;
}

/**
 * @brief  enable clock for AHB2 peripheral
 * @param  peripheral: name of the AHB2 peripheral you want enable clock
 */
void AHB2_clock_enable(AHB2_periph_t peripheral)
{
	uint32_t* RCC_AHB2ENR = (uint32_t*) (RCC_BASE_ADDR + 0x34);
	*RCC_AHB2ENR |= 1 << peripheral;
}

/**
 * @brief  enable clock for AHB1 peripheral
 * @param  peripheral: name of the AHB1 peripheral you want enable clock
 */
void AHB1_clock_enable(AHB1_periph_t peripheral)
{
	uint32_t* RCC_AHB1ENR = (uint32_t*) (RCC_BASE_ADDR + 0x30);
	*RCC_AHB1ENR |= 1 << peripheral;
}

/** SYSTEM CLOCK = 32MHz */
void RCC_Init()
{
	uint32_t* RCC_CR = (uint32_t*) (RCC_BASE_ADDR + 0x00);
	uint32_t* RCC_PLLCFGR = (uint32_t*) (RCC_BASE_ADDR + 0x04);
	uint32_t* RCC_CFGR = (uint32_t*) (RCC_BASE_ADDR + 0x08);

	/* select HSE as PLL clock entry */
	*RCC_PLLCFGR |= 1 << 22;

	/* select M = 8 -> f_PLL = 1MHz */
	*RCC_PLLCFGR &= ~(0x3F << 0);
	*RCC_PLLCFGR |= 8 << 0;

	/* select N = 64 -> f_PLL = 64MHz */
	*RCC_PLLCFGR &= ~(0x1FF << 6);
	*RCC_PLLCFGR |= 64 << 6;

	/* select P = 2 -> f_PLL = 32MHz */
	*RCC_PLLCFGR &= ~(0b11 << 16);

	/* enable HSE clock */
	*RCC_CR |= 1 << 16;

	/* wait until HSE clock is ready */
	while (((*RCC_CR >> 17) & 1) == 0);

	/* enable PLL clock */
	*RCC_CR |= 1 << 24;

	/* wait until PLL clock is ready */
	while (((*RCC_CR >> 25) & 1) == 0);

	/* select PLL as system clock */
	*RCC_CFGR &= ~(0b11 << 0);
	*RCC_CFGR |= 0b10 << 0;

	/* wait until PLL is used as the system clock */
	while (((*RCC_CFGR >> 2) & 0b11) != 0b10);
}
