#include "main.h"
#include "clock.h"
#include "timer.h"
#include "MFRC522.h"

int tmp = 0;
int main()
{
	RCC_Init();
	TIM_Init();
	SPI_Init();
	MFRC522_Init();

	while (1)
	{
		tmp = read_DATA(0x37);
		delay_sec(2);
		tmp = read_DATA(0x33);
		delay_sec(2);
		tmp = read_DATA(0x36);
		delay_sec(3);
	}
	return 0;
}
