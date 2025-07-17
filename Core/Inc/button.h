/*
 * 	File		: button.h
 *	Created on	: Jul 17, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#define IDLE 0
#define ADD  1
#define RM   2
#define GPIOC_BASE_ADDR 0x40020800
#define EXTI_BASE_ADDR  0x40013C00
//void EXTI9_5_IRQHandler();
void BUTTON_Init();
#endif /* INC_BUTTON_H_ */
