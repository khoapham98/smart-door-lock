/*
 * 	File		: timer.h
 *	Created on	: Jul 12, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_
#define TIM1_BASE_ADDR 0x40010000
void TIM1_UP_TIM10_IRQHandler();

void delay_microsec(uint32_t time);
void delay_millisec(uint32_t time);
void delay_sec(uint32_t time);
void microsec_config();
void millisec_config();
void sec_config();
void TIM_Init();
#endif /* INC_TIMER_H_ */
