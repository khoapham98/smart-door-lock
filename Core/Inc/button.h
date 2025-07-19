/*
 * 	File		: button.h
 *	Created on	: Jul 17, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_
#define NORMAL 	0x00
#define ADD  	0x01
#define REMOVE  0x02
#define PB	 	0x01
#define GPIOB_BASE_ADDR 	0x40020400
#define EXTI_BASE_ADDR  	0x40013C00
#define SYSCFG_BASE_ADDR 	0x40013800
uint8_t selectMODE();
uint8_t isPressed();
void BUTTON_Init();
void ButtonInit();
char AisPressed();
#endif /* INC_BUTTON_H_ */
