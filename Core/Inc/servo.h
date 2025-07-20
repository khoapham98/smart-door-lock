/*
 * 	File		: servo.h
 *	Created on	: Jul 19, 2025
 *	Author		: Khoa
 * 	GitHub		: https://github.com/khoapham98
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#define 	TIM2_BASE_ADDR 		0x40000000
#define 	GPIOB_BASE_ADDR		0x40020400
void door_open();
void door_close();
void SERVO_setAngle(uint8_t angle);
void SERVO_Init();
#endif /* INC_SERVO_H_ */
