/*
 * servos.h
 *
 *  Created on: Nov 13, 2025
 *      Author: joadj
 */

#ifndef INC_ACTUADORES_SERVOS_H_
#define INC_ACTUADORES_SERVOS_H_

#include "stm32f4xx_hal.h"

// Definiciones de ángulos (Ajustar mecánicamente)
#define SERVO_OPEN_ANGLE   90.0f  // Ángulo de patas abiertas
#define SERVO_CLOSE_ANGLE  0.0f   // Ángulo de patas cerradas

// Definiciones de hardware
#define SERVO_MIN_US       500    // 0 grados
#define SERVO_MAX_US       2500   // 180 grados

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} Servo_Handle_t;

void Servo_Init(Servo_Handle_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(Servo_Handle_t *servo, float angle);



#endif /* INC_ACTUADORES_SERVOS_H_ */
