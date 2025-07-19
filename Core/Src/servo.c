/*
 * 	File		: servo.c
 *	Created on	: Jul 19, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */
#include "main.h"
#include "clock.h"
#include "timer.h"
#include "servo.h"

void door_open()
{
	SERVO_setAngle(100);
	delay_sec(4);
	door_close();
}

void door_close()
{
	SERVO_setAngle(0);
}

static void SERVO_Ctrl(uint32_t pos)
{
	uint32_t* TIM2_CCR3	 = (uint32_t*) (TIM2_BASE_ADDR + 0x3C);
	*TIM2_CCR3 = pos;
}

void SERVO_setAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint32_t ccr_value;

    if (angle <= 90)
    {
        ccr_value = 2430 + ((int32_t)angle * (1450 - 2430)) / 90;
    }
    else
    {
        ccr_value = 1450 + ((int32_t)(angle - 90) * (500 - 1450)) / 90;
    }
    SERVO_Ctrl(ccr_value);
}

/**	=== TIMER 2 CHANNEL 3
 * PWM	-> PB10
 * VCC	-> 5V
 * GND	-> GND
 */
void SERVO_Init()
{
	AHB1_clock_enable(AHB1_GPIOB);
	uint32_t* GPIOB_MODER = (uint32_t*) (GPIOB_BASE + 0x00);
	uint32_t* GPIOB_AFRH  = (uint32_t*) (GPIOB_BASE + 0x24);
	/* configure PB10 as AF */
	*GPIOB_MODER &= ~(0b11 << (10 * 2));
	*GPIOB_MODER |= 0b10 << (10 * 2);
	/* select AF01 */
	*GPIOB_AFRH &= ~(0xf << 8);
	*GPIOB_AFRH |= 1 << 8;


	APB1_clock_enable(APB1_TIM2);
	uint16_t* TIM2_CR1	 = (uint16_t*) (TIM2_BASE_ADDR + 0x00);
	uint16_t* TIM2_EGR	 = (uint16_t*) (TIM2_BASE_ADDR + 0x14);
	uint16_t* TIM2_CCMR2 = (uint16_t*) (TIM2_BASE_ADDR + 0x1C);
	uint16_t* TIM2_PSC	 = (uint16_t*) (TIM2_BASE_ADDR + 0x28);
	uint32_t* TIM2_ARR 	 = (uint32_t*) (TIM2_BASE_ADDR + 0x2C);
	uint32_t* TIM2_CCR3	 = (uint32_t*) (TIM2_BASE_ADDR + 0x3C);
	uint16_t* TIM2_CCER	 = (uint16_t*) (TIM2_BASE_ADDR + 0x20);
	/* select PWM mode 1 */
	*TIM2_CCMR2 &= ~(0b111 << 4);
	*TIM2_CCMR2 |= (0b110 << 4);
	/* enable auto reload pre-load */
	*TIM2_CCMR2 |= 1 << 3;
	*TIM2_CR1 	|= 1 << 7;
	/* configure CK_CNT = 1MHz */
	*TIM2_PSC = 32 - 1;
	/* set T = 20ms (50Hz) */
	*TIM2_ARR = 20000 - 1;
	/* set servo default position is middle */
	*TIM2_CCR3 = 1500;
	/* enable channel 3 output */
	*TIM2_CCER |= 1 << 8;
	/* generate update */
	*TIM2_EGR |= 1 << 0;
	/* enable counter */
	*TIM2_CR1 |= 1 << 0;
}
