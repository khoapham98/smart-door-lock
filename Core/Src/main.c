#include "main.h"
#include "clock.h"
#include "timer.h"
#include <string.h>
#include "MFRC522.h"

int stt = 0;
uint8_t str[16];
uint8_t UIDs[4];
int main()
{
	RCC_Init();
	TIM_Init();
	SPI_Init();
	MFRC522_Init();

	while (1)
	{
		if (MFRC522_Request(PICC_REQA, str) == MI_OK)
		{
			stt = MFRC522_Anticoll(UIDs);
		}
		delay_millisec(100);
	}
	return 0;
}
