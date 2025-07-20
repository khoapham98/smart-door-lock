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
#define 	PAGE0 				0x00
#define 	PAGE1 				0x01
#define 	PAGE2 				0x02
#define 	PAGE3 				0x03
#define 	MIN_COL 			0x00
#define 	MAX_COL 			0x7F

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
void SSD1306_print8x16(char* str, uint8_t page, uint8_t col);
void SSD1306_print6x8(char* str, uint8_t page, uint8_t col);
void SSD1306_print_alphabet();
void SSD1306_ClrPage(uint8_t page);
void SSD1306_ClrScr();
void SSD1306_FillWhite();
void SSD1306_Init();
void I2C_Init();

#endif /* INC_SSD1306_H_ */
