/*
 * 	File		: mfrc522.h
 *	Created on	: Jul 12, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */

#ifndef INC_MFRC522_H_
#define INC_MFRC522_H_

#define GPIOB_BASE_ADDR 0x40020400
#define SPI1_BASE_ADDR  0x40013000
#define READ  0x80
#define WRITE 0x7E

void write_DATA(uint8_t reg_address, uint8_t data);
uint8_t read_DATA(uint8_t reg_address);
void MFRC522_reset();
void MFRC522_Init();
void SPI_Init();
#endif /* INC_MFRC522_H_ */
