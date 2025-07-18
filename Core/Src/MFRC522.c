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
uint8_t uid_cnt = 0;
static void SetBitMask(uint8_t reg_addr, uint8_t mask);
static void ClearBitMask(uint8_t reg_addr, uint8_t mask);
static void write(uint8_t reg_address, uint8_t data);
static void ClearState();
static void store_UID(uint8_t* src, uint8_t dest[][4]);
static uint8_t read(uint8_t reg_address);
static uint8_t anticoll(uint8_t *uid_out);
static uint8_t sendRequest(uint8_t reqMode);
static uint8_t send2Card(uint8_t cmd, uint8_t* _data, uint8_t datalen, uint8_t* returnData, uint32_t* returnLen);
static uint8_t UID_is_new(uint8_t* recv_buf, uint8_t uids[][4]);
static uint8_t UID_list_is_NOT_full();
static uint8_t find_UID(uint8_t* _rm, uint8_t _src[][4]);
static void remove_UID(uint8_t* rm, uint8_t src[][4]);
/* ====== Public API ====== */
/**
 * @brief  Checks whether the given UID is authorized for access
 * 		   This function compares the UID against the stored list of authorized UIDs
 * @param  uid_list: A 2D array to store registered UIDs
 * @return 1: The UID is authorized
 * 		   0: Otherwise
 */
uint8_t MFRC522_IsValidUID(uint8_t uid_list[][4])
{
	uint8_t tmp[4] = {0};
	anticoll(tmp);
	if (!UID_is_new(tmp, uid_list))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * @brief  Removes a UID from the list if it exists
 * @param  uid_list: A 2D array to store registered UIDs
 * @return 1: if a UID was successfully removed
 * 		   0: if the UID was not found or the list is empty
 */
uint8_t MFRC522_RemoveUID(uint8_t uid_list[][4])
{
	uint8_t tmp[4] = {0};
	anticoll(tmp);
	if ((UID_is_new(tmp, uid_list)) || (uid_cnt == 0))
	{
		return 0;
	}
	else
	{
		remove_UID(tmp, uid_list);
	}
	return 1;
}

/**
 * @brief  Checks for a new UID and stores it if not already in the list
 * @param  uid_list: A 2D array to store registered UIDs
 * @return 1: if a new UID was successfully stored
 * 		   0: if it already exists or the list is full
 */
uint8_t MFRC522_CheckAndStoreUID(uint8_t uid_list[][4])
{
	uint8_t tmp[4] = {0};
	anticoll(tmp);
	if (UID_is_new(tmp, uid_list) && UID_list_is_NOT_full())
	{
		store_UID(tmp, uid_list);
		return 1;
	}
	return 0;
}

/**
 * @brief  Checks whether an RFID tag/card is present near the reader
 * @return 1: The tag/card is near
 * 		   0: Otherwise
 */
uint8_t MFRC522_IsTagPresent()
{
	if (sendRequest(PICC_REQA) == MI_OK)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/* ====== BCC & UID Management ====== */
static void remove_UID(uint8_t* rm, uint8_t src[][4])
{
	int index = find_UID(rm, src);
	for (int r = index; r < uid_cnt; r++)
	{
		for (int c = 0; c < 4; c++)
		{
			src[r][c] = src[r + 1][c];
		}
	}
	uid_cnt--;
}

static uint8_t find_UID(uint8_t* _rm, uint8_t _src[][4])
{
	int r = 0;
	for (; r < uid_cnt; r++)
	{
		int found = 1;
		for (int c = 0; c < 4; c++)
		{
			if (_rm[c] != _src[r][c])
			{
				found = 0;
				break;
			}
		}
		if (found) break;
	}
	return r;
}

static uint8_t UID_is_new(uint8_t* recv_buf, uint8_t uids[][4])
{
	for (int r = 0; r < uid_cnt; r++)
	{
		int uid_existed = 1;
		for (int c = 0; c < 4; c++)
		{
			if (recv_buf[c] != uids[r][c])
			{
				uid_existed = 0;
				break;
			}
		}
		if (uid_existed) return 0;
	}
	return 1;
}

static uint8_t UID_list_is_NOT_full()
{
	return (uid_cnt < (MAX_UIDs - 1)) ? 1 : 0;
}

static void store_UID(uint8_t* src, uint8_t dest[][4])
{
	for (int i = 0; i < 4; i++)
	{
		dest[uid_cnt][i] = src[i];
	}
	uid_cnt++;
}

static uint8_t check_BCC(uint8_t* uid)
{
	uint8_t bcc = 0;
	for (int i = 0; i < 4; i++)
	{
		bcc ^= uid[i];
	}
	return (bcc == uid[4]) ? MI_OK : MI_ERR;
}

/**
 * @brief  Sends a REQA command to detect if a card is present
 * @param  reqMode The request mode: REQA (0x26) or WUPA (0x52)
 * @param  TagType Buffer to store the 2-byte ATQA response
 * @return MI_OK : if a card is detected
 * 		   MI_ERR: otherwise
 */
static uint8_t sendRequest(uint8_t reqMode)
{
	uint8_t tmp[2] = { 0 };
	uint8_t status;
	uint32_t backlen;
	ClearState();
	write(BitFramingReg, 0x07);
	tmp[0] = reqMode;

	status = send2Card(PCD_TRANSCEIVE, tmp, 1, tmp, &backlen);
	if ((status != MI_OK) || (backlen != 2))
	{
		status = MI_ERR;
	}

	return status;
}

static uint8_t anticoll(uint8_t* uid_out)
{
    uint8_t status;
    uint32_t unLen;
    uint8_t cmd_buffer[2] = { PICC_ANTICOLL, 0x20 };
    uint8_t recv_buffer[5];

    ClearState();
    write(BitFramingReg, 0x00);

    status = send2Card(PCD_TRANSCEIVE, cmd_buffer, 2, recv_buffer, &unLen);

    if (status == MI_OK && unLen == 5 && check_BCC(recv_buffer) == MI_OK)
    {
		for (int i = 0; i < 4; i++)
		{
			uid_out[i] = recv_buffer[i];
		}
    }

    return status;
}

/* ====== Core Communication ====== */
static uint8_t send2Card(uint8_t cmd, uint8_t* _data, uint8_t datalen, uint8_t* returnData, uint32_t* returnLen)
{
	uint8_t irq = 0;
	if (cmd == PCD_TRANSCEIVE) 		irq = 0x30;
	else if (cmd == PCD_AUTHENT) 	irq = 0x10;

	write(ComIEnReg, irq | 0x80); 	/* enable interrupt */
	write(ComIrqReg, 0x7F);			/* clear all interrupt flags */
	write(FIFOLevelReg, 0x80);		/* clear FIFO */
	write(CommandReg, PCD_IDLE);

	/* write data into FIFO */
	for (int i = 0; i < datalen; i++)
	{
		write(FIFODataReg, _data[i]);
	}

	/* send command */
	write(CommandReg, cmd);
	if (cmd == PCD_TRANSCEIVE)
	{
		SetBitMask(BitFramingReg, 0x80); /* start send data from FIFO */
	}

	uint16_t timeout = 10000;
	while (timeout-- && !(read(ComIrqReg) & irq));
	if (timeout == 0)
	{
		ClearState();
		return MI_ERR;
	}

	/* check for error */
	if (read(ErrorReg) & 0x1B)	{ return MI_ERR; }
	ClearBitMask(BitFramingReg, 0x80);

	/* receive data from tag */
	if (cmd == PCD_TRANSCEIVE)
	{
		*returnLen = read(FIFOLevelReg);
		for (int i = 0; i < *returnLen; i++)
		{
			returnData[i] = read(FIFODataReg);
		}
	}
	return MI_OK;
}

static void ClearState()
{
    write(CommandReg, PCD_IDLE);
    write(ComIrqReg, 0x7F);         // clear interrupt flags
    write(FIFOLevelReg, 0x80);      // clear FIFO
    ClearBitMask(BitFramingReg, 0x07);      // clear bit framing
}

/* ====== SPI & Slave/Chip Select ===== */
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
 * @brief  This function is used to write 1 byte data into any register of MRFC522
 * @param  reg_address: address of the register to write data to
 * 		   data: data want to write
 */
static void write(uint8_t reg_address, uint8_t data)
{
	select_MFRC522();
	SPI_Transmit(reg_address, data);
	NOT_select_MFRC522();
}

/**
 * @brief  This function is used to read 1 byte data from any register of MRFC522
 * @param  reg_address: address of the register to read data from
 */
static uint8_t read(uint8_t reg_address)
{
	select_MFRC522();
	uint8_t val = SPI_Receive(reg_address);
	NOT_select_MFRC522();
	return val;
}

static void ClearBitMask(uint8_t reg_addr, uint8_t mask)
{
    uint8_t tmp = read(reg_addr);
    write(reg_addr, tmp & (~mask));  /* clear bit mask */
}

static void SetBitMask(uint8_t reg_addr, uint8_t mask)
{
    uint8_t tmp = read(reg_addr);
    write(reg_addr, tmp | mask);  /* set bit mask */
}

static void AntennaOFF()
{
	ClearBitMask(TxControlReg, 0x03);
}

static void AntennaON()
{
	SetBitMask(TxControlReg, 0x03);
}

/* ====== Configuration & Initialization ====== */
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
	write(CommandReg, 0x0F);	/* soft reset */
	write(TModeReg, 0x80);		/* auto-start */
	write(TPrescalerReg, 0xA9);	/* f_timer = 13.56e6/(2 * A9h) ~= 39.882KHz -> T_timer ~= 25us */
	write(TReloadRegH, 0x03); /* set reload value = 03E8h = 1000 */
	write(TReloadRegL, 0xE8);

	/* used more */
//	MFRC522_write(TPrescalerReg, 0x3E);
//	MFRC522_write(TReloadRegH, 0);
//	MFRC522_write(TReloadRegL, 30);

	write(TxASKReg, 0x40);		/* force 100% ASK */
	write(ModeReg, 0x3D);
	AntennaON();
	delay_millisec(10);
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
