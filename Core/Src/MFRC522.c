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
static void SetBitMask(uint8_t reg_addr, uint8_t mask);
static void ClearBitMask(uint8_t reg_addr, uint8_t mask);
static uint8_t MFRC522_send2Card(uint8_t cmd, uint8_t* _data, uint8_t datalen, uint8_t* returnData, uint32_t* returnLen);

uint8_t MFRC522_Anticoll(uint8_t *serNum)
{
	uint8_t status;
    uint8_t i;
    uint8_t serNumCheck=0;
    uint32_t unLen;

	MFRC522_write(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;

    status = MFRC522_send2Card(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);
    if (status == MI_OK)
	{
    	 //Check card serial number
		for (i=0; i<4; i++)
		{
		 	serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i])
		{
			status = MI_ERR;
		}
    }

    return status;
}

uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *TagType)
{
	uint8_t status;
	uint32_t backBits;	// The received data bits

	MFRC522_write(BitFramingReg, 0x07);		//TxLastBists = BitFramingReg[2..0]
	TagType[0] = reqMode;

	status = MFRC522_send2Card(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
	if ((status != MI_OK) || (backBits != 0x10))
	{
		status = MI_ERR;
	}

	return status;
}

static uint8_t MFRC522_send2Card(uint8_t cmd, uint8_t* _data, uint8_t datalen, uint8_t* returnData, uint32_t* returnLen)
{
	uint8_t irq = 0;
	if (cmd == PCD_TRANSCEIVE) irq = 0x30;
	else if (cmd == PCD_AUTHENT) irq = 0x10;

	MFRC522_write(ComIEnReg, irq | 0x80); 	/* enable interrupt */
	MFRC522_write(ComIrqReg, 0x7F);			/* clear all interrupt flags */
	MFRC522_write(FIFOLevelReg, 0x80);		/* clear FIFO */
	MFRC522_write(CommandReg, PCD_IDLE);

	/* write data into FIFO */
	for (int i = 0; i < datalen; i++)
	{
		MFRC522_write(FIFODataReg, _data[i]);
	}

	/* send command */
	MFRC522_write(CommandReg, cmd);
	if (cmd == PCD_TRANSCEIVE)
	{
		SetBitMask(BitFramingReg, 0x80); /* start send data from FIFO */
	}

	uint16_t timeout = 10000;
	while (timeout-- && !(MFRC522_read(ComIrqReg) & irq));
	if (timeout == 0) return MI_ERR;

	/* check for error */
	if (MFRC522_read(ErrorReg) & 0x1B)	{ return MI_ERR; }
	ClearBitMask(BitFramingReg, 0x80);

	/* receive data from tag */
	if (cmd == PCD_TRANSCEIVE)
	{
		*returnLen = MFRC522_read(FIFOLevelReg);
		for (int i = 0; i < *returnLen; i++)
		{
			returnData[i] = MFRC522_read(FIFODataReg);
		}
	}
	return MI_OK;
}

void AntennaOFF()
{
	ClearBitMask(TxControlReg, 0x03);
}

void AntennaON()
{
	SetBitMask(TxControlReg, 0x03);
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
	/* configure MFRC522 */
	MFRC522_write(CommandReg, 0x0F);	/* soft reset */
	MFRC522_write(TModeReg, 0x80);		/* auto-start */
	MFRC522_write(TPrescalerReg, 0xA9);	/* f_timer = 13.56e6/(2 * A9h) ~= 39.882KHz -> T_timer ~= 25us */
	MFRC522_write(TReloadRegH, 0x03); /* set reload value = 03E8h = 1000 */
	MFRC522_write(TReloadRegL, 0xE8);

	/* used more */
//	MFRC522_write(TPrescalerReg, 0x3E);
//	MFRC522_write(TReloadRegH, 0);
//	MFRC522_write(TReloadRegL, 30);

	MFRC522_write(TxASKReg, 0x40);		/* force 100% ASK */
	MFRC522_write(ModeReg, 0x3D);
	AntennaON();
	delay_millisec(10);
}

static void ClearBitMask(uint8_t reg_addr, uint8_t mask)
{
    uint8_t tmp = MFRC522_read(reg_addr);
    MFRC522_write(reg_addr, tmp & (~mask));  /* clear bit mask */
}

static void SetBitMask(uint8_t reg_addr, uint8_t mask)
{
    uint8_t tmp = MFRC522_read(reg_addr);
    MFRC522_write(reg_addr, tmp | mask);  /* set bit mask */
}

/**
 * @brief  This function is used to write 1 byte data into any register of MRFC522
 * @param  reg_address: address of the register to write data to
 * 		   data: data want to write
 */
void MFRC522_write(uint8_t reg_address, uint8_t data)
{
	select_MFRC522();
	SPI_Transmit(reg_address, data);
	NOT_select_MFRC522();
}

/**
 * @brief  This function is used to read 1 byte data from any register of MRFC522
 * @param  reg_address: address of the register to read data from
 */
uint8_t MFRC522_read(uint8_t reg_address)
{
	select_MFRC522();
	uint8_t val = SPI_Receive(reg_address);
	NOT_select_MFRC522();
	return val;
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
