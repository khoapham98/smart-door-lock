/*
 * 	File		: stm32_uart.c
 *	Created on	: Jul 21, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */
#include "main.h"
#include "clock.h"
#include "stm32_uart.h"
static uint32_t get_size(char* s);

void send_msg(char* str)
{
	UART_sendString(str);
}

void UART_sendString(char* str)
{
	uint32_t strlen = get_size(str);
	for (int i = 0; i < strlen; i++)
	{
		UART_sendChar(str[i]);
	}
}

static uint32_t get_size(char* s)
{
	uint32_t cnt = 0;
	while (s[cnt] != 0) cnt++;
	return cnt;
}

void UART_sendChar(uint8_t data)
{
	uint8_t* USART_DR  = (uint8_t*)  (USART6_BASE_ADDR + 0x04);
	uint32_t* USART_SR = (uint32_t*) (USART6_BASE_ADDR + 0x00);
	*USART_DR = data;
	while (((*USART_SR >> 7) & 1) == 0);
}

/**  UART6
 * PC6		-> TX
 * Parity 	-> Even
 * Baud Rate-> 9600bps
 */
void UART_Init()
{
	AHB1_clock_enable(AHB1_GPIOC);
	uint32_t* GPIOC_MODER = (uint32_t*) (GPIOC_BASE_ADDR + 0x00);
	uint32_t* GPIOC_AFRL  = (uint32_t*) (GPIOC_BASE_ADDR + 0x20);
	/* configure PC6 as AF */
	*GPIOC_MODER &= ~(0b11 << (6 * 2));
	*GPIOC_MODER |= 0b10 << (6 * 2);
	/* select AF08 */
	*GPIOC_AFRL &= ~(0xf << (6 * 4));
	*GPIOC_AFRL |= 8 << (6 * 4);

	APB2_clock_enable(APB2_USART6);
	uint32_t* USART_CR1 = (uint32_t*) (USART6_BASE_ADDR + 0x0C);
	uint32_t* USART_BRR = (uint32_t*) (USART6_BASE_ADDR + 0x08);
	/* set word length = 9 data bits including parity */
	*USART_CR1 |= 1 << 12;
	/* enable even parity */
	*USART_CR1 |= 1 << 10;
	/* enable transmitter */
	*USART_CR1 |= 1 << 3;
	/* configure baud rate = 9600bps */
	*USART_BRR = (208 << 4) | (5 << 0);
	/* enable USART */
	*USART_CR1 |= 1 << 13;
}
