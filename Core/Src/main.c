#include "main.h"
#include "clock.h"
#include "timer.h"
#include "mfrc522.h"

void LED_Init()
{
	AHB1_clock_enable(AHB1_GPIOD);
	uint32_t* GPIOD_MODER = (uint32_t*) (GPIOD_BASE + 0x00);
	*GPIOD_MODER &= ~(0b11 << (15 * 2));
	*GPIOD_MODER |= (0b01 << (15 * 2));
}
void LED_Ctrl(char on)
{
	uint32_t* GPIOD_ODR = (uint32_t*) (GPIOD_BASE + 0x14);
	if (on)
	{
		*GPIOD_ODR |= 1 << 15;
	}
	else
	{
		*GPIOD_ODR &= ~(1 << 15);
	}
}
int tmp;
int main()
{
	RCC_Init();
	SPI_Init();
	TIM_Init();
	LED_Init();
	while (1)
	{
//		tmp = MFRC522_readDATA(0x37);
		LED_Ctrl(1);
		delay_millisec(2000);
		LED_Ctrl(0);
		delay_millisec(4000);
	}
	return 0;
}
