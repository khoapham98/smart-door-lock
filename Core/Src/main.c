#include "main.h"
#include "clock.h"
#include "timer.h"
#include <string.h>
#include "MFRC522.h"
int stt = 0;
uint8_t str[16];
uint8_t IDs[5];
int main()
{
	RCC_Init();
	TIM_Init();
	SPI_Init();
	MFRC522_Init();

	while (1)
	{
		stt = MFRC522_Request(PICC_REQA, str);
		stt = MFRC522_Anticoll(str);
		memcpy(IDs, str, 5);
		delay_sec(1);
	}
	return 0;
}
