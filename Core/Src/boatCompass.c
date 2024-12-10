/*
 * boatCompass.c
 *
 *  Created on: Dec 8, 2024
 *      Author: Tarcisio
 */

#include "boatCompass.h"

// fatores de correção determinados na calibração
int16_t xMin, yMin, xMax, yMax;
float escX = 1.0;
float escY = 1.0;
int16_t offX = 0;
int16_t offY = 0;

// Diferença entre o Polo Magnético e o Geográfico
float declination = 0;

// seta os parâmetros iniciais para calibração
void startCompassCalibration() {
  xMax = yMax = -32768;
  xMin = yMin = 32767;
}

// seta os parâmetros finais para calibração
void endCompassCalibration(){
	 // Offset para centralizar leituras em zero
	  offX = (xMax + xMin) / 2;
	  offY = (yMax + yMin) / 2;

	  // Escala para ter a mesma variação nos dois eixos
	  int16_t varX = xMax - xMin;
	  int16_t varY = yMax - yMin;
	  if (varY > varX) {
	    escY = 1.0;
	    escX = (float) varY / varX;
	  } else {
	    escX = 1.0;
	    escY = (float) varX / varY;
	  }
}

// mostra o ângulo para o qual o barco está apontando, sendo 0º o Norte
int16_t readDirection(void) {
  int16_t x, y, z;
  int16_t PI = 3.14;

  HMC5883L_getHeading(&x, &y, &z);

  // Registra mínimo e máximo para a calibração
  if (x < xMin) {
    xMin = x;
  }
  if (xMax < x) {
    xMax = x;
  }
  if (y < yMin) {
    yMin = y;
  }
  if (yMax < y) {
    yMax = y;
  }

  // corrige e calcula o angulo em radianos
  float xC = (x - offX) * escX;
  float yC = (y - offY) * escY;
  float angulo = atan2 (xC, yC) + declination;

  // Garante que está entre 0 e 2*PI
  if (angulo < 0) {
    angulo += 2.0 * PI;
  } else if (angulo >= 2 * PI) {
    angulo -= 2.0 * PI;
  }

  // Converte para graus
  return round (360 - ((angulo * 180.0) / PI));
}

// realiza o processo de calibração
void processCompassCalibration(){
    int pos = 0;

    // Inicia o processo de calibração da bússola
    startCompassCalibration();

//    HMC5883L_setMode(HMC5883L_MODE_CONTINUOUS);
    // Calcula o tempo final para a calibração (20 segundos a partir de agora)
    uint32_t tmpFim = HAL_GetTick() + 10000;

    // Loop de calibração
    while (HAL_GetTick() < tmpFim) {
        // Lê a direção da bússola
    	readDirection();

        // Incrementa a posição
        pos += 5;
        if (pos == 360) {
            pos = 0;
        }

        // Aguarda 10ms
        HAL_Delay(10);
    }

    // Finaliza a calibração
    endCompassCalibration();
}
