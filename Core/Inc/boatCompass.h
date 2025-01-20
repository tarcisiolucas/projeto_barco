/*
 * boatCompass.h
 *
 *  Created on: Dec 8, 2024
 *      Authors:
 *      - Tarcisio Lucas Cairo Carvalho
 *      - Filipe Campos Reis
 *      - Pedro de Araujo Cruz
 *
 *  Description:
 *  This header file defines the interfaces for the compass module used in the boat navigation system.
 *  It includes functions for initializing, calibrating, and reading data from the compass sensor.
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


/**
 * @brief Inicia o processo de calibração do sensor de bússola.
 *
 * Configura o sensor para entrar no modo de calibração, permitindo que ele registre as variações necessárias para corrigir
 * os erros relacionados ao ambiente ou posicionamento.
 */
void startCompassCalibration();

/**
 * @brief Processa a calibração da bússola.
 *
 * Deve ser chamada repetidamente durante o processo de calibração para registrar e processar dados do sensor.
 * Isso permite ao sensor coletar informações suficientes para ajustar sua precisão.
 */
void processCompassCalibration();

/**
 * @brief Finaliza o processo de calibração da bússola.
 *
 * Salva os dados da calibração no sensor ou em um local apropriado, concluindo o processo de calibração.
 * Essa função deve ser chamada após o ambiente ter sido varrido completamente.
 */
void endCompassCalibration();

/**
 * @brief Lê a direção atual da bússola em graus.
 *
 * Retorna o ângulo de direção calculado pelo sensor de bússola, em graus, onde:
 * - 0 graus aponta para o norte.
 * - 90 graus aponta para o leste.
 * - 180 graus aponta para o sul.
 * - 270 graus aponta para o oeste.
 *
 * @return int16_t A direção atual em graus (0 a 360).
 */
int16_t readDirection(void);


#endif /* INC_BOATCOMPASS_H_ */
