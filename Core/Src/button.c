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

char ispressed()
{
	uint32_t* GPIOC_IDR = (uint32_t*) (GPIOC_BASE_ADDR + 0x10);
	if ((((*GPIOC_IDR >> 8) & 1) == 0) || (((*GPIOC_IDR >> 9) & 1) == 0))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
void EXTI9_5_IRQHandler()	// fixme
{
	uint32_t* EXTI_PR = (uint32_t*) (EXTI_BASE_ADDR + 0x14);
	if (ispressed())
	{
		if (((*EXTI_PR >> 8) & 1) == 1)		/* if interrupt signal is detected on PC8 */
		{
			button_state = ADD;
			*EXTI_PR = 1 << 8;				/* clear interrupt flag on PC8 */
		}
		else if (((*EXTI_PR >> 9) & 1) == 1)		/* if interrupt signal is detected on PC8 */
		{
			button_state = RM;
			*EXTI_PR = 1 << 9;				/* clear interrupt flag on PC9 */
		}
		*EXTI_PR |= 0b11 << 8;
	}
}

void EXTI0_IRQHandler()
{
	uint32_t* EXTI_PR = (uint32_t*) (EXTI_BASE_ADDR + 0x14);
	button_state = ADD;
	*EXTI_PR |= 1 << 0;
}

void BUTTON_Init()
{
	AHB1_clock_enable(AHB1_GPIOC);
	AHB1_clock_enable(AHB1_GPIOA);
	uint32_t* GPIOC_MODER = (uint32_t*) (GPIOC_BASE_ADDR + 0x00);
	uint32_t* EXTI_IMR    = (uint32_t*) (EXTI_BASE_ADDR + 0x00);
	uint32_t* EXTI_RTSR   = (uint32_t*) (EXTI_BASE_ADDR + 0x08);
	uint32_t* EXTI_FTSR   = (uint32_t*) (EXTI_BASE_ADDR + 0x0C);
	uint32_t* NVIC_ISER0  = (uint32_t*) (0xE000E100);

//	/* configure PC8 & PC9 as INPUT */
//	*GPIOC_MODER &= ~((0b11 << (8 * 2)) | (0b11 << (9 * 2)));
//
//
//	/* enable rising trigger for line 8 & 9 */
//	*EXTI_RTSR |= (1 << 8) | (1 << 9);
//	*EXTI_FTSR |= (1 << 8) | (1 << 9);
//
//	/* enable interrupt mask register for line 8 & 9 */
//	*EXTI_IMR |= (1 << 8) | (1 << 9);
//
//	/* enable interrupt set */
//	*NVIC_ISER0 |= 1 << 23;

	uint32_t* GPIOA_MODER = (uint32_t*) (GPIOA_BASE + 0x00);
	*GPIOA_MODER &= ~(0b11 << 0);
	*EXTI_FTSR |= (1 << 0);
	*EXTI_IMR |= (1 << 0);
	*NVIC_ISER0 |= 1 << 6;
}

