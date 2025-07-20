/*
 * 	File		: button.c
 *	Created on	: Jul 17, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */
#include "main.h"
#include "clock.h"
#include "timer.h"
#include "ssd1306.h"
#include "button.h"
uint8_t button_state = NORMAL;

static void delay_ms(uint32_t ms)
{
	for (uint32_t j = 0; j < ms; j++)
	{
		for (volatile uint32_t i = 0; i < 8000; i++);
	}
}

static uint8_t isPressed()
{
	uint32_t* GPIOB_IDR = (uint32_t*) (GPIOB_BASE_ADDR + 0x10);
	if (((*GPIOB_IDR >> 1) & 1) == 1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static uint8_t button_cnt = 0;
static uint8_t selectMODE()
{
	uint8_t mode;
	int time = 1;
	while (isPressed())
	{
		button_cnt++;
		delay_ms(50);
		if (isPressed()) time = 2;
	}

	if (button_cnt > 0 && button_cnt <= 4)
	{
		if (time == 1)
		{
			mode = ADD;
		}
		else if (time == 2)
		{
			mode = REMOVE;
		}
	}
	else
	{
		mode = NORMAL;
	}
	button_cnt = 0;
	return mode;
}

/**
 * One click 	: Add UID
 * Double click : Remove UID
 * Press & hold : IDLE mode (open the door)
 */
void EXTI1_IRQHandler()
{
	uint32_t* EXTI_PR = (uint32_t*) (EXTI_BASE_ADDR + 0x14);
	button_state = selectMODE();
	switch (button_state)
	{
		case NORMAL:
			SSD1306_print6x8(".ACCESS MODE.", PAGE0, 0x18);
			break;
		case ADD:
			SSD1306_print6x8(".ENROLL MODE.", PAGE0, 0x18);
			break;
		case REMOVE:
			SSD1306_print6x8(".REMOVE MODE.", PAGE0, 0x18);
			break;
		default:
			break;
	}
	*EXTI_PR |= 1 << 1;
}

/**
 * @brief  Configure PB1 for buttons and use external interrupt to detect when the button is pressed
 * PB1	-> OUT
 * GND	-> GND
 * VCC	-> 3V
 */
void BUTTON_Init()
{
	APB2_clock_enable(APB2_SYSCFG);
	uint32_t* GPIOB_MODER = (uint32_t*) (GPIOB_BASE_ADDR + 0x00);
	uint32_t* EXTI_IMR    = (uint32_t*) (EXTI_BASE_ADDR  + 0x00);
	uint32_t* EXTI_RTSR   = (uint32_t*) (EXTI_BASE_ADDR  + 0x08);
	uint32_t* NVIC_ISER0  = (uint32_t*) (0xE000E100);
	uint32_t* SYSCFG_EXTICR1 = (uint32_t*) (SYSCFG_BASE_ADDR + 0x08);

	/* configure PB1 as INPUT */
	*GPIOB_MODER &= ~(0b11 << (1 * 2));

	/* select Port B for EXTI interrupt line 1 */
	*SYSCFG_EXTICR1 |= (PB << 4);

	/* configure EXTI & NVIC */
	*EXTI_RTSR  |= (1 << 1);
	*EXTI_IMR   |= (1 << 1);
	*NVIC_ISER0 |= (1 << 7);
}
