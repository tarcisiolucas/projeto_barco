/*
 * boatMotor.h
 *
 *  Created on: Dec 8, 2024
 *      Author: Tarcisio
 */

#ifndef INC_BOATENGINE_H_
#define INC_BOATENGINE_H_

#include "stm32f4xx_hal.h"

void changeDCMotorDirection(char d);
void changeDCMotorSpeed(uint16_t speed, TIM_HandleTypeDef htim);

#endif /* INC_BOATENGINE_H_ */
