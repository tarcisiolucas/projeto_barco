/*
 * boatEngine.c
 *
 *  Created on: Dec 8, 2024
 *      Author: Tarcisio
 */

#include "boatEngine.h"

void changeDCMotorSpeed(uint16_t speed, TIM_HandleTypeDef htim)
{
  if (speed > 16800)
    speed = 16800;
  __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_2, speed);
}

void changeDCMotorDirection(char d)
{
  if (d == 'f')
  {
    // Vetor dos bits a serem enviados
    uint8_t data_bits[] = {0, 1, 0, 1, 0, 1, 0, 1};

    // Ativar troca dos valores
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

    // Loop para enviar os bits
    for (uint8_t i = 0; i < 8; i++)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, data_bits[i]); // Configura o dado
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);            // Pulso de clock
      HAL_Delay(1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // Finaliza o clock
    }

    // Atualizar saída
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
  }
  else if (d == 't')
  {
    // Vetor dos bits a serem enviados
    uint8_t data_bits[] = {1, 0, 1, 0, 1, 0, 1, 0};

    // Ativar as saidas
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

    // Loop para enviar os bits
    for (uint8_t i = 0; i < 8; i++)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, data_bits[i]); // Configura o dado
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);            // Pulso de clock
      HAL_Delay(1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // Finaliza o clock
    }

    // Atualizar saída
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
  }
}
