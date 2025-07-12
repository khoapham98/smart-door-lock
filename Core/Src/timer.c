/*
 * 	File		: timer.c
 *	Created on	: Jul 12, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */
#include "main.h"
#include "clock.h"
#include "timer.h"
static int cnt = 0;

void delay_microsec(uint32_t time)
{
	microsec_config();
	cnt = 0;
	while (cnt <= time);
}

void delay_millisec(uint32_t time)
{
	millisec_config();
	cnt = 0;
	while (cnt < time);
}

void delay_sec(uint32_t time)
{
	sec_config();
	cnt = 0;
	while (cnt < time);
}

void TIM1_UP_TIM10_IRQHandler()
{
	uint16_t* TIM1_SR  = (uint16_t*) (TIM1_BASE_ADDR + 0x10);
	uint16_t* TIM1_CNT = (uint16_t*) (TIM1_BASE_ADDR + 0x24);

	/* reset counter value */
	*TIM1_CNT = 0;
	while ((*TIM1_SR & 1) == 0);
	*TIM1_SR &= ~1;
	cnt++;
}

void microsec_config()
{
	uint16_t* TIM1_PSC = (uint16_t*) (TIM1_BASE_ADDR + 0x28);
	uint16_t* TIM1_ARR = (uint16_t*) (TIM1_BASE_ADDR + 0x2C);
	uint16_t* TIM1_EGR = (uint16_t*) (TIM1_BASE_ADDR + 0x14);
	/* set CK_CNT = 1kHz */
	*TIM1_PSC = 1 - 1;

	/* set auto-reload value */
	*TIM1_ARR = 32 - 1;

	/* (force) generate update */
	*TIM1_EGR |= 1 << 0;
}

void millisec_config()
{
	uint16_t* TIM1_PSC = (uint16_t*) (TIM1_BASE_ADDR + 0x28);
	uint16_t* TIM1_ARR = (uint16_t*) (TIM1_BASE_ADDR + 0x2C);
	uint16_t* TIM1_EGR = (uint16_t*) (TIM1_BASE_ADDR + 0x14);
	/* set CK_CNT = 1kHz */
	*TIM1_PSC = 32 - 1;

	/* set auto-reload value */
	*TIM1_ARR = 1000 - 1;

	/* (force) generate update */
	*TIM1_EGR |= 1 << 0;
}

void sec_config()
{
	uint16_t* TIM1_PSC = (uint16_t*) (TIM1_BASE_ADDR + 0x28);
	uint16_t* TIM1_ARR = (uint16_t*) (TIM1_BASE_ADDR + 0x2C);
	uint16_t* TIM1_EGR = (uint16_t*) (TIM1_BASE_ADDR + 0x14);
	/* set CK_CNT = 1kHz */
	*TIM1_PSC = 32000 - 1;

	/* set auto-reload value */
	*TIM1_ARR = 1000 - 1;

	/* (force) generate update */
	*TIM1_EGR |= 1 << 0;
}

/**
 * @brief  this function is used to initial timer1
 */
void TIM_Init()
{
	APB2_clock_enable(APB2_TIM1);
	uint16_t* TIM1_PSC  = (uint16_t*) (TIM1_BASE_ADDR + 0x28);
	uint16_t* TIM1_DIER = (uint16_t*) (TIM1_BASE_ADDR + 0x0C);
	uint16_t* TIM1_CR1  = (uint16_t*) (TIM1_BASE_ADDR + 0x00);
	uint16_t* TIM1_ARR  = (uint16_t*) (TIM1_BASE_ADDR + 0x2C);
	uint32_t* NVIC_ISER0 = (uint32_t*) (0xE000E100);

	/* set CK_CNT = 1kHz */
	*TIM1_PSC = 32000 - 1;

	/* set auto-reload value */
	*TIM1_ARR = 1000;

	/* enable update interrupt */
	*TIM1_DIER |= 1 << 0;

	/* interrupt set-enable */
	*NVIC_ISER0 |= 1 << 25;

	/* enable counter */
	*TIM1_CR1 |= 1 << 0;
}
