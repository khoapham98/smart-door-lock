/*
 * oled.h
 *
 *  Created on: Jun 20, 2025
 *      Author: ACER
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#define 	I2C3_BASE_ADDR 		0x40005C00
#define 	GPIOA_BASE_ADDR 	0x40020000
#define 	GPIOC_BASE_ADDR 	0x40020800
#define 	SSD1306_ADDR 		0x3C
#define 	PAGE 				4
#define 	COLUMN 				128

typedef enum
{
	W, R
} mode_t;

typedef enum
{
	ACCESS, ENROLL, RMOVE
} RFID_mode_t;
typedef enum
{
	CMD = 0x00,
	DATA = 0x40
} ctrl_t;
void SSD1306_print_status(char* str);
void SSD1306_gotoxy(uint8_t page_start, uint8_t page_end, uint8_t col_start, uint8_t col_end);
void SSD1306_print_mode(RFID_mode_t mode);
void SSD1306_print_string(char* str);
void SSD1306_print_alphabet();
void SSD1306_FillWhite();
void SSD1306_ClrScr();
void SSD1306_Init();
void I2C_Init();

#endif /* INC_SSD1306_H_ */
