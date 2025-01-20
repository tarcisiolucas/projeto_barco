/*
 * boatMotor.h
 *
 *  Created on: Jan 19, 2025
 *      Authors:
 *      - Tarcisio Lucas Cairo Carvalho
 *      - Filipe Campos Reis
 *      - Pedro de Araujo Cruz
 *
 *  Description:
 *  This header file defines the interfaces for controlling the DC motor used in the boat's propulsion system.
 *  It includes functions to change the motor's direction and adjust its speed through PWM control.
 */


#ifndef INC_BOATENGINE_H_
#define INC_BOATENGINE_H_

#include "stm32f4xx_hal.h"

/**
 * @brief Altera a direção do motor de corrente contínua (DC).
 *
 * Esta função configura os pinos de controle do motor para alterar sua direção de rotação
 * (ex.: horário ou anti-horário).
 *
 * @param d Um caractere indicando a direção desejada:
 *          - 'F': Frente (Forward)
 *          - 'B': Trás (Backward)
 *          - 'S': Parar (Stop)
 */
void changeDCMotorDirection(char d);

/**
 * @brief Altera a velocidade do motor de corrente contínua (DC).
 *
 * Configura o PWM de um timer para ajustar a velocidade do motor. A velocidade é
 * controlada através do duty cycle do PWM.
 *
 * @param speed Valor da velocidade (duty cycle) no intervalo de 0 a 1000.
 *              - 0: Motor parado.
 *              - 1000: Velocidade máxima.
 * @param htim Estrutura do tipo TIM_HandleTypeDef que representa o timer configurado
 *             para controlar o PWM do motor.
 */
void changeDCMotorSpeed(uint16_t speed, TIM_HandleTypeDef htim);

#endif /* INC_BOATENGINE_H_ */
