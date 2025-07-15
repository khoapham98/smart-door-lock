#include "main.h"
#include "clock.h"
#include "timer.h"
#include "MFRC522.h"
int tmp = 0;
uint8_t str[16];
int main()
{
	RCC_Init();
	TIM_Init();
	SPI_Init();
	MFRC522_Init();

	while (1)
	{
		tmp = MFRC522_Request(PICC_REQIDL, str);
		tmp = MFRC522_Anticoll(str);
		delay_sec(3);
	}
	return 0;
}
