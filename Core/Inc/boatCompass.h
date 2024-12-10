/*
 * boatCompass.h
 *
 *  Created on: Dec 8, 2024
 *      Author: Tarcisio
 */

#ifndef INC_BOATCOMPASS_H_
#define INC_BOATCOMPASS_H_

#include "stm32f4xx_hal.h"
#include "HMC5883L.h"
#include "math.h"

// fatores de correção determinados na calibração
extern int16_t xMin, yMin, xMax, yMax;
extern float escX;
extern float escY;
extern int16_t offX;
extern int16_t offY;

// Diferença entre o Polo Magnético e o Geográfico
extern float declination;


void endCompassCalibration();
int16_t readDirection(void);
void startCompassCalibration();
void processCompassCalibration();


#endif /* INC_BOATCOMPASS_H_ */
