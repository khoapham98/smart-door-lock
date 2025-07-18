/*
 * 	File		: button.c
 *	Created on	: Jul 17, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */
#include "main.h"
#include "clock.h"
#include "button.h"

uint8_t button_state = IDLE;

void EXTI1_IRQHandler()
{
	uint32_t* EXTI_PR = (uint32_t*) (EXTI_BASE_ADDR + 0x14);
	button_state = ADD;
	*EXTI_PR |= 1 << 1;
}

void EXTI2_IRQHandler()
{
	uint32_t* EXTI_PR = (uint32_t*) (EXTI_BASE_ADDR + 0x14);
	button_state = RM;
	*EXTI_PR |= 1 << 2;
}

/**
 * @brief  Configure PB1 and PB2 for buttons and use external interrupt to detect when the button is pressed
 */
void BUTTON_Init()
{
	AHB1_clock_enable(AHB1_GPIOB);
	APB2_clock_enable(APB2_SYSCFG);
	uint32_t* GPIOB_MODER = (uint32_t*) (GPIOB_BASE_ADDR + 0x00);
	uint32_t* GPIOB_PUPDR = (uint32_t*) (GPIOB_BASE_ADDR + 0x0C);
	uint32_t* EXTI_IMR    = (uint32_t*) (EXTI_BASE_ADDR  + 0x00);
	uint32_t* EXTI_RTSR   = (uint32_t*) (EXTI_BASE_ADDR  + 0x08);
	uint32_t* NVIC_ISER0  = (uint32_t*) (0xE000E100);
	uint32_t* SYSCFG_EXTICR1 = (uint32_t*) (SYSCFG_BASE_ADDR + 0x08);

	/* configure PB1 & PB2 as INPUT */
	*GPIOB_MODER &= ~((0b11 << (1 * 2)) | (0b11 << (2 * 2)));

	/* configure pull-up resister for PB1 & PB2 */
	*GPIOB_PUPDR &= ~((0b11 << (1 * 2)) | (0b11 << (2 * 2)));
	*GPIOB_PUPDR |= (0b01 << (1 * 2)) | (0b01 << (2 * 2));

	/* select Port B for EXTI interrupt */
	*SYSCFG_EXTICR1 |= (PB << 4) | (PB << 8);

	/* configure EXTI & NVIC */
	*EXTI_RTSR  |= (1 << 1) | (1 << 2);
	*EXTI_IMR   |= (1 << 1) | (1 << 2);
	*NVIC_ISER0 |= (1 << 7) | (1 << 8);
}

