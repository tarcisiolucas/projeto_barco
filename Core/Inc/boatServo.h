/*
 * boatServo.h
 *
 *  Created on: Dec 8, 2024
 *      Author: Tarcisio
 */

#ifndef INC_BOATSERVO_H_
#define INC_BOATSERVO_H_

#include "stm32f4xx_hal.h"

void setPWM(TIM_HandleTypeDef, uint32_t, uint16_t, uint16_t);
void setServoPosition(uint16_t angle, TIM_HandleTypeDef htim);

#endif /* INC_BOATSERVO_H_ */
