#include "main.h"
#include "clock.h"
#include "timer.h"
#include "MFRC522.h"

uint8_t str[16];
uint8_t uid_ls[MAX_UIDs][4];
int check;
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
			check = MFRC522_CheckAndStoreUID(uid_ls);
		}
		delay_millisec(100);
	}
	return 0;
}
