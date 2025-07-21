/*
 * 	File		: stm32_uart.h
 *	Created on	: Jul 21, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */

#ifndef INC_STM32_UART_H_
#define INC_STM32_UART_H_
#define		USART6_BASE_ADDR	0x40011400
#define 	GPIOC_BASE_ADDR		0x40020800
void send_msg(char* str);
void UART_sendString(char* str);
void UART_sendChar(uint8_t data);
void UART_Init();
#endif /* INC_STM32_UART_H_ */
