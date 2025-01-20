/*
 * boatServo.c
 *
 *  Created on: Dec 8, 2024
 *      Author: Tarcisio
 */

#include "boatServo.h"

void setPWM(TIM_HandleTypeDef* timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
  HAL_TIM_PWM_Stop(timer, channel); // stop generation of pwm
  TIM_OC_InitTypeDef sConfigOC;
  timer->Init.Period = period; // set the period duration
  HAL_TIM_PWM_Init(timer);   // reinititialise with new period value
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse; // set the pulse duration
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, channel);
  HAL_TIM_PWM_Start(timer, channel); // start pwm generation
}

void setServoPosition(uint16_t angle, TIM_HandleTypeDef* htim)
{
  // Define pulse and angle limits
  uint16_t pulseMin = 44;
  uint16_t pulseMax = 144;
  uint16_t angleMin = 0;
  uint16_t angleMax = 180;

  // Ensure the angle is within limits
  if (angle > angleMax)
  {
    angle = angleMax;
  }

  if (angle < angleMin)
  {
    angle = angleMin;
  }

  // Convert the angle to pulse value
  uint16_t pulse = pulseMin + ((pulseMax - pulseMin) * angle) / (angleMax - angleMin);

  setPWM(htim, TIM_CHANNEL_1, 12500, pulse);
}
