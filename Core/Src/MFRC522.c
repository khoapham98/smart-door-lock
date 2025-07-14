/*
 * 	File		: mfrc522.c
 *	Created on	: Jul 12, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */
#include "main.h"
#include "clock.h"
#include "timer.h"
#include "MFRC522.h"
static void select_MFRC522();
static void NOT_select_MFRC522();
static void SPI_Transmit(uint8_t reg_addr, uint8_t _data);
static uint8_t SPI_Receive(uint8_t reg_addr);

/**
 * @brief  This function is used to write 1 byte data into any register of MRFC522
 * @param  reg_address: address of the register to write data to
 * 		   data: data want to write
 */
void write_DATA(uint8_t reg_address, uint8_t data)
{
	select_MFRC522();
	SPI_Transmit(reg_address, data);
	NOT_select_MFRC522();
}

/**
 * @brief  This function is used to read 1 byte data from any register of MRFC522
 * @param  reg_address: address of the register to read data from
 */
uint8_t read_DATA(uint8_t reg_address)
{
	select_MFRC522();
	uint8_t val = SPI_Receive(reg_address);
	NOT_select_MFRC522();
	return val;
}

void MFRC522_reset()
{
	uint32_t* GPIOB_ODR = (uint32_t*) (GPIOB_BASE_ADDR + 0x14);
	*GPIOB_ODR &= ~(1 << 7);
	delay_microsec(100);
	*GPIOB_ODR |= 1 << 7;
	delay_millisec(50);
}

void MFRC522_Init()
{
	MFRC522_reset();
}

static uint8_t SPI_Receive(uint8_t reg_addr)
{
	uint8_t* SPI_DR = (uint8_t*) (SPI1_BASE_ADDR  + 0x0C);
	uint16_t* SPI_SR = (uint16_t*) (SPI1_BASE_ADDR  + 0x08);

	/* data send sequence */
	while (((*SPI_SR >> 1) & 1) == 0);	/* wait until the TX buffer is empty */
	*SPI_DR = READ | (reg_addr << 1);	/* send register address */
	while (((*SPI_SR >> 7) & 1) == 1);	/* wait until SPI is free */

	/* data read sequence */
	while ((*SPI_SR & 1) == 0);			/* wait until the RX buffer is not empty */
	uint8_t dummy = *SPI_DR;			/* read dummy data */

	/* data send sequence */
	while (((*SPI_SR >> 1) & 1) == 0);	/* wait until the TX buffer is empty */
	*SPI_DR = 0x00;						/* send dummy address */
	while (((*SPI_SR >> 7) & 1) == 1);	/* wait until SPI is free */

	/* data read sequence */
	while ((*SPI_SR & 1) == 0);			/* wait until the RX buffer is not empty */
	uint8_t data = *SPI_DR;				/* read data */

	return data;
}

static void SPI_Transmit(uint8_t reg_addr, uint8_t _data)
{
	uint8_t* SPI_DR = (uint8_t*) (SPI1_BASE_ADDR  + 0x0C);
	uint16_t* SPI_SR = (uint16_t*) (SPI1_BASE_ADDR  + 0x08);

	/* data send sequence */
	while (((*SPI_SR >> 1) & 1) == 0);	/* wait until the TX buffer is empty */
	*SPI_DR = (reg_addr << 1) & WRITE;	/* send register address */
	while (((*SPI_SR >> 7) & 1) == 1);	/* wait until SPI is free */

	/* data read sequence */
	while ((*SPI_SR & 1) == 0);			/* wait until the RX buffer is not empty */
	uint8_t dummy = *SPI_DR;			/* read dummy data */

	/* data send sequence */
	while (((*SPI_SR >> 1) & 1) == 0);	/* wait until the TX buffer is empty */
	*SPI_DR = _data;						/* send dummy address */
	while (((*SPI_SR >> 7) & 1) == 1);	/* wait until SPI is free */

	/* data read sequence */
	while ((*SPI_SR & 1) == 0);			/* wait until the RX buffer is not empty */
	dummy = *SPI_DR;					/* read dummy data */
}

/**
 * brief  This function is used to select slave by pull SS to LOW
 */
static void select_MFRC522()
{
	uint32_t* GPIOB_ODR = (uint32_t*) (GPIOB_BASE_ADDR + 0x14);
	*GPIOB_ODR &= ~(1 << 6);
}

/**
 * brief  This function is used to select slave by set SS to HIGH
 */
static void NOT_select_MFRC522()
{
	uint32_t* GPIOB_ODR = (uint32_t*) (GPIOB_BASE_ADDR + 0x14);
	*GPIOB_ODR |= 1 << 6;
}

/**
 * @brief  RFID-MFRC522 initial function
 * PB3	-> SCK
 * PB4  -> MISO
 * PB5	-> MOSI
 * PB6	-> SDA (NSS)
 * PB7	-> RST
 * GND	-> GND
 * VCC	-> 3V
 */
void SPI_Init()
{
	AHB1_clock_enable(AHB1_GPIOB);
	uint32_t* GPIOB_MODER = (uint32_t*) (GPIOB_BASE_ADDR + 0x00);
	uint32_t* GPIOB_ODR   = (uint32_t*) (GPIOB_BASE_ADDR + 0x14);
	uint32_t* GPIOB_AFRL  = (uint32_t*) (GPIOB_BASE_ADDR + 0x20);

	/* set PB6,7 as OUTPUT, PB3,4,5 as AF */
	*GPIOB_MODER &= ~(0x3ff << (3 * 2));
	*GPIOB_MODER |= (0b10 << (3 * 2)) | (0b10 << (4 * 2)) | (0b10 << (5 * 2)) | (0b01 << (6 * 2)) | (0b01 << (7 * 2));

	/* pull NSS (PB6) and RST (PB7) to HIGH */
	*GPIOB_ODR |= (1 << 6) | (1 << 7);

	/* select AF05 for PB3,4,5 */
	*GPIOB_AFRL &= ~( (0xf << (3 * 4)) | (0xf << (4 * 4)) | (0xf << (5 * 4)) );
	*GPIOB_AFRL |= (5 << (3 * 4)) | (5 << (4 * 4)) | (5 << (5 * 4));

	APB2_clock_enable(APB2_SPI1);
	uint16_t* SPI_CR1 = (uint16_t*) (SPI1_BASE_ADDR + 0x00);

	/* set STM32 as master */
	*SPI_CR1 |= 1 << 2;

	/* set baud rate = 1Mbps (MRFC522 can handle up to 10Mbps) */
	*SPI_CR1 &= ~(0b111 << 3);
	*SPI_CR1 |= 0b100 << 3;

	/* software slave management */
	*SPI_CR1 |= 1 << 9;
	*SPI_CR1 |= 1 << 8;

	/* enable SPI */
	*SPI_CR1 |= 1 << 6;
}
