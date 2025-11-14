/*
 * servos.c
 *
 *  Created on: Nov 13, 2025
 *      Author: joadj
 */
#include "servos.h"

void Servo_Init(Servo_Handle_t *servo, TIM_HandleTypeDef *htim, uint32_t channel) {
    servo->htim = htim;
    servo->channel = channel;
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

void Servo_SetAngle(Servo_Handle_t *servo, float angle) {
    if(angle < 0.0f) angle = 0.0f;
    if(angle > 180.0f) angle = 180.0f;

    // Mapeo lineal de Ángulo a Microsegundos
    uint16_t pulse = (uint16_t)(SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US) / 180.0f));

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}

