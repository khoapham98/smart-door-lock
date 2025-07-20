#include "main.h"
#include "clock.h"
#include "timer.h"
#include "MFRC522.h"
#include "button.h"
#include "servo.h"
#include "ssd1306.h"

uint8_t uid_ls[MAX_UIDs][4];
extern uint8_t button_state;

int main()
{
	RCC_Init();
	TIM_Init();
	SPI_Init();
	I2C_Init();
	BUTTON_Init();
	MFRC522_Init();
	SERVO_Init();
	SSD1306_Init();

	SSD1306_default_mode();
	while (1)
	{
		if (MFRC522_IsTagPresent())
		{
			switch (button_state)
			{
				case NORMAL:

					if (MFRC522_IsValidUID(uid_ls))
					{
						SSD1306_print8x16("ACCESS GRANTED", PAGE2, 8);
						door_open();
					}
					else
					{
						SSD1306_print8x16("ACCESS DENIED", PAGE2, 12);
						delay_sec(2);
					}
					SSD1306_print8x16("SCAN YOUR TAG", PAGE2, 12);
					break;
				case ADD:
					if (MFRC522_CheckAndStoreUID(uid_ls) == 1)
					{
						SSD1306_print8x16("ENROLL SUCCESS", PAGE2, 8);
					}
					else
					{
						SSD1306_print8x16("ENROLL FAILED", PAGE2, 12);
					}
					delay_sec(2);
					SSD1306_print8x16("SCAN YOUR TAG", PAGE2, 12);
					break;
				case REMOVE:
					if (MFRC522_RemoveUID(uid_ls) == 1)
					{
						SSD1306_print8x16("REMOVE SUCCESS", PAGE2, 8);
					}
					else
					{
						SSD1306_print8x16("REMOVE FAILED", PAGE2, 12);
					}
					delay_sec(2);
					SSD1306_print8x16("SCAN YOUR TAG", PAGE2, 12);
					break;
				default:
					break;
			}
		}
	}
	return 0;
}
