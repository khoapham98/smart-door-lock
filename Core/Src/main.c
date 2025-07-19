#include "main.h"
#include "clock.h"
#include "timer.h"
#include "MFRC522.h"
#include "button.h"

uint8_t uid_ls[MAX_UIDs][4];
extern uint8_t uid_cnt;
extern uint8_t button_state;
extern int button_cnt;

int main()
{

	HAL_Init();
	RCC_Init();
	TIM_Init();
	SPI_Init();
	BUTTON_Init();
	MFRC522_Init();

	while (1)
	{
		if (MFRC522_IsTagPresent())
		{
			switch (button_state) {
				case NORMAL:
					if (MFRC522_IsValidUID(uid_ls))
					{
						// control motor to open the door
					}
					break;
				case ADD:
					MFRC522_CheckAndStoreUID(uid_ls);
					break;
				case REMOVE:
					MFRC522_RemoveUID(uid_ls);
					break;
				default:
					break;
			}
		}
		delay_millisec(200);
	}
	return 0;
}



