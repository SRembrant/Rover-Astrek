/*
 * Control_Pose.h
 *
 *  Created on: Oct 24, 2025
 *      Author: joadj
 *      Descripcion: Controlador PID de alto nivel para seguimiento de waypoint
 */

#ifndef INC_ACTUADORES_CONTROL_POSE_H_
#define INC_ACTUADORES_CONTROL_POSE_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Constantes de configuración
#define POSE_CONTROL_FREQUENCY_HZ    20.0f    // Frecuencia de control (20Hz)
#define POSE_CONTROL_DT              0.05f    // Periodo (1/20 = 50ms)

// Umbrales de control
#define POSE_DISTANCE_THRESHOLD      0.3f     // 30cm - considerado "llegado"
#define POSE_ANGLE_THRESHOLD         0.17f    // ~10 grados en radianes
#define POSE_ANGLE_TOLERANCE         0.087f   // ~5 grados - tolerancia para avanzar

// Límites de comandos
#define POSE_MAX_LINEAR_VELOCITY     0.5f     // 0.5 m/s máximo
#define POSE_MAX_ANGULAR_VELOCITY    0.52f    // ~30 deg/s en rad/s

// Estructura de PID
typedef struct {
    // Ganancias
    float Kp;
    float Ki;
    float Kd;

    // Estados internos
    float error_prev;
    float integral;
    float integral_max;  // Anti-windup

    // Límites de salida
    float output_min;
    float output_max;
} PID_Controller_t;

// Estructura del controlador de pose
typedef struct {
    // PIDs
    PID_Controller_t pid_linear;    // Control de distancia
    PID_Controller_t pid_angular;   // Control de orientación

    // Estado actual (entrada del controlador)
    float x_actual;          // metros
    float y_actual;          // metros
    float theta_actual;      // radianes (-π a π)

    // Target (objetivo)
    float x_target;
    float y_target;
    float theta_target;      // Opcional, para orientación final

    // Comandos de salida
    float vx_cmd;           // Velocidad lineal (m/s)
    float wz_cmd;           // Velocidad angular (rad/s)

    // Estado del controlador
    bool target_reached;
    bool orientation_aligned;
    float distance_to_target;
    float angle_error;

    // Modo de control
    bool pure_pursuit_mode;  // true: sigue waypoint, false: orientación fija

} PoseController_t;

// Funciones públicas
void PoseController_Init(PoseController_t *ctrl);
void PoseController_SetTarget(PoseController_t *ctrl, float x_target, float y_target);
void PoseController_Update(PoseController_t *ctrl, float x_actual, float y_actual, float theta_actual);
void PoseController_Reset(PoseController_t *ctrl);

// Utilidades
float normalize_angle(float angle);
float angle_difference(float target, float current);

#endif /* INC_ACTUADORES_CONTROL_POSE_H_ */
