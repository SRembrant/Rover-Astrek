/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c (VERSIÓN CORREGIDA)
 * Description        : Code for freertos applications
 ******************************************************************************
 */
/* USER CODE END Header */
//
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define _USE_MATH_DEFINES
#include <math.h>

#include "GPS.h"
#include "IMU.h"
#include "Serial.h"
#include "sr04.h"

#include "Navegacion.h"
//#include "NavGlobal.h"
#include "Taquito.h"

#include "usart.h"
#include "i2c.h"

#include <string.h>
#include <stdio.h>

#include "Control_Rover.h"
#include "Control_Pose.h"
#include "Control_Kinematics.h"
#include "Control_PWM.h"
#include "servos.h"

#include "Sensors_I2C.h"
#include "LoRa_RYLR998.h"
#include "datalogger.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern Rover_Config Rover;
extern HCSR04_Config_t hcsr04_config;
extern GPS_Config_t gps_config;
extern MPU9250_Data IMU_Data;
volatile float g_current_lux = 0.0f;
extern TIM_HandleTypeDef htim2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LUX_THRESHOLD_OPEN  500.0f // Abrir si hay más luz que esto
#define LUX_HYSTERESIS      50.0f  // Margen para evitar rebotes
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

uint32_t raw_light_debug = 0;
static PoseController_t pose_controller;
static PWM_Compensation_t pwm_compensation;
static uint8_t control_initialized = 0;

Servo_Handle_t hServoDiag1; // Par 1 (ej. FR + BL)
Servo_Handle_t hServoDiag2; // Par 2 (ej. FL + BR)

/* USER CODE END Variables */
/* Definitions for Navegacion */
osThreadId_t NavegacionHandle;
const osThreadAttr_t Navegacion_attributes = {
		.name = "Navegacion",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Taquito */
osThreadId_t TaquitoHandle;
const osThreadAttr_t Taquito_attributes = {
		.name = "Taquito",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NavGlobal */
osThreadId_t NavGlobalHandle;
const osThreadAttr_t NavGlobal_attributes = {
		.name = "NavGlobal",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for Control */
osThreadId_t ControlHandle;
const osThreadAttr_t Control_attributes = {
		.name = "Control",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Ultrasonido */
osThreadId_t UltrasonidoHandle;
const osThreadAttr_t Ultrasonido_attributes = {
		.name = "Ultrasonido",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityBelowNormal2,
};
/* Definitions for Geoposicion */
osThreadId_t GeoposicionHandle;
const osThreadAttr_t Geoposicion_attributes = {
		.name = "Geoposicion",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Transmision */
osThreadId_t TransmisionHandle;
const osThreadAttr_t Transmision_attributes = {
		.name = "Transmision",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Sensores_I2C */
osThreadId_t Sensores_I2CHandle;
const osThreadAttr_t Sensores_I2C_attributes = {
		.name = "Sensores_I2C",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for logTask */
osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = {
		.name = "logTask",
		.stack_size = 2048 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Servos */
osThreadId_t ServosHandle;
const osThreadAttr_t Servos_attributes = {
		.name = "Servos",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensorDataQueue */
osMessageQueueId_t sensorDataQueueHandle;
const osMessageQueueAttr_t sensorDataQueue_attributes = {
		.name = "sensorDataQueue"
};
/* Definitions for gpsDataQueue */
osMessageQueueId_t gpsDataQueueHandle;
const osMessageQueueAttr_t gpsDataQueue_attributes = {
		.name = "gpsDataQueue"
};
/* Definitions for controlDataQueue */
osMessageQueueId_t controlDataQueueHandle;
const osMessageQueueAttr_t controlDataQueue_attributes = {
		.name = "controlDataQueue"
};
/* Definitions for navStatesQueue */
osMessageQueueId_t navStatesQueueHandle;
const osMessageQueueAttr_t navStatesQueue_attributes = {
		.name = "navStatesQueue"
};
/* Definitions for hcsr04DataQueue */
osMessageQueueId_t hcsr04DataQueueHandle;
const osMessageQueueAttr_t hcsr04DataQueue_attributes = {
		.name = "hcsr04DataQueue"
};
/* Definitions for logQueue */
osMessageQueueId_t logQueueHandle;
const osMessageQueueAttr_t logQueue_attributes = {
		.name = "logQueue"
};
/* Definitions for hcsr04Semaphore */
osSemaphoreId_t hcsr04SemaphoreHandle;
const osSemaphoreAttr_t hcsr04Semaphore_attributes = {
		.name = "hcsr04Semaphore"
};
/* Definitions for serialSemaphore */
osSemaphoreId_t serialSemaphoreHandle;
const osSemaphoreAttr_t serialSemaphore_attributes = {
		.name = "serialSemaphore"
};
/* Definitions for imuSemaphore */
osSemaphoreId_t imuSemaphoreHandle;
const osSemaphoreAttr_t imuSemaphore_attributes = {
		.name = "imuSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void Rover_SetPWM_Differential(Rover_Config *rover, PWM_Commands_t commands)
{
	// ========== MOTORES DERECHOS ==========
	if (commands.dir_right == 0) {
		__HAL_TIM_SET_COMPARE(rover->f_right_motor.pwm_timer,
				rover->f_right_motor.pwm_channel_IN1,
				commands.pwm_right);
		__HAL_TIM_SET_COMPARE(rover->f_right_motor.pwm_timer,
				rover->f_right_motor.pwm_channel_IN2, 0);

		__HAL_TIM_SET_COMPARE(rover->b_right_motor.pwm_timer,
				rover->b_right_motor.pwm_channel_IN1,
				commands.pwm_right);
		__HAL_TIM_SET_COMPARE(rover->b_right_motor.pwm_timer,
				rover->b_right_motor.pwm_channel_IN2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(rover->f_right_motor.pwm_timer,
				rover->f_right_motor.pwm_channel_IN1, 0);
		__HAL_TIM_SET_COMPARE(rover->f_right_motor.pwm_timer,
				rover->f_right_motor.pwm_channel_IN2,
				commands.pwm_right);

		__HAL_TIM_SET_COMPARE(rover->b_right_motor.pwm_timer,
				rover->b_right_motor.pwm_channel_IN1, 0);
		__HAL_TIM_SET_COMPARE(rover->b_right_motor.pwm_timer,
				rover->b_right_motor.pwm_channel_IN2,
				commands.pwm_right);
	}

	// ========== MOTORES IZQUIERDOS ==========
	if (commands.dir_left == 0) {
		__HAL_TIM_SET_COMPARE(rover->f_left_motor.pwm_timer,
				rover->f_left_motor.pwm_channel_IN1,
				commands.pwm_left);
		__HAL_TIM_SET_COMPARE(rover->f_left_motor.pwm_timer,
				rover->f_left_motor.pwm_channel_IN2, 0);

		__HAL_TIM_SET_COMPARE(rover->b_left_motor.pwm_timer,
				rover->b_left_motor.pwm_channel_IN1,
				commands.pwm_left);
		__HAL_TIM_SET_COMPARE(rover->b_left_motor.pwm_timer,
				rover->b_left_motor.pwm_channel_IN2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(rover->f_left_motor.pwm_timer,
				rover->f_left_motor.pwm_channel_IN1, 0);
		__HAL_TIM_SET_COMPARE(rover->f_left_motor.pwm_timer,
				rover->f_left_motor.pwm_channel_IN2,
				commands.pwm_left);

		__HAL_TIM_SET_COMPARE(rover->b_left_motor.pwm_timer,
				rover->b_left_motor.pwm_channel_IN1, 0);
		__HAL_TIM_SET_COMPARE(rover->b_left_motor.pwm_timer,
				rover->b_left_motor.pwm_channel_IN2,
				commands.pwm_left);
	}
}

/* USER CODE END FunctionPrototypes */

void NavegacionTask(void *argument);
void TaquitoTask(void *argument);
void Navegacion_Global(void *argument);
void ControlTask(void *argument);
void UltrasonidoTask(void *argument);
void GPSTask(void *argument);
void TransmisionTask(void *argument);
void SensoresTask(void *argument);
void Datos_SD(void *argument);
void PatasRoverTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of hcsr04Semaphore */
	hcsr04SemaphoreHandle = osSemaphoreNew(1, 0, &hcsr04Semaphore_attributes);

	/* creation of serialSemaphore */
	serialSemaphoreHandle = osSemaphoreNew(1, 1, &serialSemaphore_attributes);

	/* creation of imuSemaphore */
	imuSemaphoreHandle = osSemaphoreNew(1, 1, &imuSemaphore_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of sensorDataQueue */
	sensorDataQueueHandle = osMessageQueueNew (5, sizeof(SensorData_t), &sensorDataQueue_attributes);

	/* creation of gpsDataQueue */
	gpsDataQueueHandle = osMessageQueueNew (5, sizeof(GPS_Data_t), &gpsDataQueue_attributes);

	/* creation of controlDataQueue */
	controlDataQueueHandle = osMessageQueueNew (5, sizeof(ControlCommand_t), &controlDataQueue_attributes);

	/* creation of navStatesQueue */
	navStatesQueueHandle = osMessageQueueNew (5, sizeof(evento_navegacion), &navStatesQueue_attributes);

	/* creation of hcsr04DataQueue */
	hcsr04DataQueueHandle = osMessageQueueNew (5, sizeof(HCSR04_Data_t), &hcsr04DataQueue_attributes);

	/* creation of logQueue */
	logQueueHandle = osMessageQueueNew (5, sizeof(Datalog_Entry_t), &logQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of Navegacion */
	//NavegacionHandle = osThreadNew(NavegacionTask, NULL, &Navegacion_attributes);

	/* creation of Taquito */
	TaquitoHandle = osThreadNew(TaquitoTask, NULL, &Taquito_attributes);

	/* creation of NavGlobal */
	//NavGlobalHandle = osThreadNew(Navegacion_Global, NULL, &NavGlobal_attributes);

	/* creation of Control */
	ControlHandle = osThreadNew(ControlTask, NULL, &Control_attributes);

	/* creation of Ultrasonido */
	UltrasonidoHandle = osThreadNew(UltrasonidoTask, NULL, &Ultrasonido_attributes);

	/* creation of Geoposicion */
	GeoposicionHandle = osThreadNew(GPSTask, NULL, &Geoposicion_attributes);

	/* creation of Transmision */
	TransmisionHandle = osThreadNew(TransmisionTask, NULL, &Transmision_attributes);

	/* creation of Sensores_I2C */
	Sensores_I2CHandle = osThreadNew(SensoresTask, NULL, &Sensores_I2C_attributes);

	/* creation of logTask */
	logTaskHandle = osThreadNew(Datos_SD, NULL, &logTask_attributes);

	/* creation of Servos */
	ServosHandle = osThreadNew(PatasRoverTask, NULL, &Servos_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_NavegacionTask */
/**
 * @brief  Function implementing the Navegacion thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_NavegacionTask */
void NavegacionTask(void *argument)
{
	/* USER CODE BEGIN NavegacionTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END NavegacionTask */
}

/* USER CODE BEGIN Header_TaquitoTask */
/**
 * @brief Function implementing the Taquito thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TaquitoTask */
void TaquitoTask(void *argument)
{
	/* USER CODE BEGIN TaquitoTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END TaquitoTask */
}

/* USER CODE BEGIN Header_Navegacion_Global */
/**
 * @brief Function implementing the NavGlobal thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Navegacion_Global */
void Navegacion_Global(void *argument)
{
	/* USER CODE BEGIN Navegacion_Global */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END Navegacion_Global */
}

/* USER CODE BEGIN Header_ControlTask */
/**
 * @brief Function implementing the Control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument)
{


    // ===== INICIALIZACIÓN =====
    osDelay(500);  // ← AÑADIDO: Esperar a que otras tareas inicialicen

    if (!control_initialized) {
        PoseController_Init(&pose_controller);
        PWM_InitCompensation(&pwm_compensation);
        control_initialized = 1;

        // Mensaje de inicio SIN bloqueo
        HAL_UART_Transmit(&huart1, (uint8_t*)"[CONTROL] Inicializado\r\n", 24, 100);
    }

    // ===== Variables persistentes para MODE_POSE_GIRO =====
    typedef enum { GYRO_IDLE = 0, GYRO_ROTATING } GyroState_t;
    static GyroState_t giro_state = GYRO_IDLE;

    static uint32_t giro_start_tick = 0;
    static const uint32_t GIRO_TIMEOUT = 15000;
    static float yaw_integrado = 0.0f;     // integración del gyro
	static uint32_t last_tick = 0;         // para integrar dt
	const float GIRO_WZ = 0.35f;           // velocidad angular de giro
    const float GIRO_OBJ_RAD = 90.0f * M_PI / 180.0f;  // 90 grados

    // Variables locales
    static uint8_t target_set = 0;
    static uint16_t telemetry_counter = 0;

    GPS_Data_t gps_data;
    ControlCommand_t cmd;
    osStatus_t status;
	Sensors_I2C_Handle_t hsensors = {0};

    // Variables de trabajo para el control
	float vx_cmd = 0.0f;
	float wz_cmd = 0.0f;

    // ===== BUCLE PRINCIPAL =====
    for(;;)
    {
    	 // 1. Leer GPS actual
		status = osMessageQueueGet(gpsDataQueueHandle, &gps_data, NULL, 10);
		if (status != osOK) continue;

		// Intentar leer un comando de la cola de Taquito (no bloqueante, timeout 0)
		status = osMessageQueueGet(controlDataQueueHandle, &cmd, NULL, 0);
		if (status != osOK) continue;

		// 2. Convertir GPS a coordenadas locales (usar gpsACartesiano)
		P_Cartesiano pos_actual = gpsACartesiano(estacionTerrena, &gps_data);

		// 3. Obtener orientación (temporalmente de GPS course)
		float theta_actual = deg2rad(gps_data.course);

        // ----- ACTUALIZAR CONTROLADOR -----
        PoseController_Update(&pose_controller, pos_actual.x, pos_actual.y, theta_actual);

        // ----- VERIFICAR SI LLEGAMOS -----
        if (pose_controller.target_reached) {
            PWM_Commands_t stop_cmd = {0, 0, 0, 0};
            Rover_SetPWM_Differential(&Rover, stop_cmd);

            //HAL_UART_Transmit(&huart1, (uint8_t*)"[CONTROL] Target alcanzado!\r\n", 30, 100);

            // Resetear controlador
            PoseController_Reset(&pose_controller);

            target_set = 0;
            osDelay(2000);

            continue;
        }


        // 4. SELECCIÓN DE LÓGICA DE CONTROL BASADA EN EL MODO
       		switch (cmd.mode)
       		{
       			case MODE_WALL_FOLLOW: //control PID para el seguimiento de pared
       				// Taquito ya calculó el PID lateral y dio el vx y wz (corrección)
       				vx_cmd = cmd.target_vx;
       				wz_cmd = cmd.target_wz;

       				// Resetear el PID de pose (para que no interfiera)
       				PoseController_Reset(&pose_controller);
       				break;

       			case MODE_POSE_GIRO:

       				    // ============================================================
       				    //  1) SI EL GIRO NO HA INICIADO --> inicializar integración
       				    // ============================================================
       				    if (giro_state == GYRO_IDLE)
       				    {
       				        yaw_integrado = 0.0f;
       				        last_tick = HAL_GetTick();
       				        giro_start_tick = last_tick;

       				        giro_state = GYRO_ROTATING;

       				        // dirección de giro
       				        vx_cmd = 0.0f;
       				        wz_cmd = (cmd.giro_dir == GIRO_ANTIHORARIO) ? GIRO_WZ : -GIRO_WZ;

       				        break; // salir del switch
       				    }

       				    // ==========================================
       				    //       2) SI EL GIRO ESTÁ EN CURSO
       				    // ==========================================
       				    if (giro_state == GYRO_ROTATING)
       				    {
       				        bool done = false;

       				        // ----- Integrar gyro -----
       				        uint32_t now = HAL_GetTick();
       				        float dt = (now - last_tick) / 1000.0f;  // convertir a segundos
       				        last_tick = now;

       				        // IMPORTANTE: gyro_z debe estar en rad/s
       				        status = osMessageQueueGet(sensorDataQueueHandle, &hsensors, NULL, 0);
       				     	if (status != osOK) continue;

       				        float gyro_z = hsensors.data.gyro[3];   // Asegúrate: gz YA normalizado en rad/s dentro del driver

       				        yaw_integrado += gyro_z * dt;

       				        // ----- Verificar si ya alcanzó 90° -----
       				        if (cmd.giro_dir == GIRO_ANTIHORARIO)
       				        {
       				            if (yaw_integrado >= GIRO_OBJ_RAD)
       				                done = true;
       				        }
       				        else // GIRO_HORARIO
       				        {
       				            if (yaw_integrado <= -GIRO_OBJ_RAD)
       				                done = true;
       				        }

       				        // ----- Timeout -----
       				        if ((now - giro_start_tick) > GIRO_TIMEOUT)
       				            done = true;

       				        // ----- Si terminó el giro -----
       				        if (done)
       				        {
       				            PWM_Commands_t stop = {0};
       				            Rover_SetPWM_Differential(&Rover, stop);

       				            giro_state = GYRO_IDLE;

       				            PoseController_Reset(&pose_controller);
       				            break;
       				        }

       				        // ----- Si NO ha terminado → seguir girando -----
       				        vx_cmd = 0.0f;
       				        wz_cmd = (cmd.giro_dir == GIRO_ANTIHORARIO) ? GIRO_WZ : -GIRO_WZ;
       				        break;
       				    }
       				    break;

       			case MODE_POSE_TARGET: //la logica que ya se tenia, avanza hacia un target x,y
       				// Comportamiento de control de pose original (ir a X, Y)
       				PoseController_Update(&pose_controller,
       									 pos_actual.x, pos_actual.y, theta_actual,
       									 cmd.target_x, cmd.target_y); // Usar targets del comando

       				vx_cmd = pose_controller.vx_cmd;
       				wz_cmd = pose_controller.wz_cmd;
       				break;

       			case MODE_FORWARD:
       				//avance "en bruto" desde taquito
       				vx_cmd = cmd.target_vx;
       				wz_cmd = cmd.target_wz;
       				break;
       		}

        // ----- CINEMÁTICA Y PWM -----
        WheelVelocities_t wheel_velocities = Kinematics_Inverse(
            vx_cmd,
            wz_cmd
        );

        PWM_Commands_t pwm_commands = PWM_FromWheelVelocities(
            wheel_velocities,
            &pwm_compensation
        );

        Rover_SetPWM_Differential(&Rover, pwm_commands);

        // ----- TELEMETRÍA REDUCIDA (cada 2 segundos) -----
        telemetry_counter++;
        if (telemetry_counter >= 40) {  // 40 * 50ms = 2 segundos
            telemetry_counter = 0;

            // Telemetría compacta
            char telem[128];
            snprintf(telem, sizeof(telem),
                    "Pos:(%.2f,%.2f) Dist:%.2fm PWM:(%u,%u)\r\n",
                    sim_x, sim_y,
                    pose_controller.distance_to_target,
                    pwm_commands.pwm_right, pwm_commands.pwm_left);

            HAL_UART_Transmit(&huart1, (uint8_t*)telem, strlen(telem), 100);
        }

        osDelay(50);  // 20Hz
    }
	
}

/* USER CODE BEGIN Header_UltrasonidoTask */
/**
 * @brief Function implementing the Ultrasonido thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UltrasonidoTask */
void UltrasonidoTask(void *argument)
{
	/* USER CODE BEGIN UltrasonidoTask */

	HCSR04_Data_t data_ultrasonico;

	/* Infinite loop */
	for(;;)
	{
		// 1. Llama a la función de lectura.
		// Esta función bloqueará la tarea (espera el semáforo)
		// hasta que la interrupción del timer reciba el eco.
		HAL_StatusTypeDef status = HCSR04_ReadDistance(&data_ultrasonico);
		// 2. Comprobar si la lectura fue exitosa
		if (status == HAL_OK)
		{
			// 3. Imprimir el dato usando la función de Serial.c
			// Esta función ya comprueba si data.is_valid
			Serial_PrintHCSR04Data(&data_ultrasonico);
		}
		else
		{
			// 4. (Opcional) Imprimir si hubo un error (timeout)
			//Serial_PrintString("[Ultrasonico] Error: Timeout (eco no recibido)\r\n");
		}
		// 5. Esperar antes de la próxima medición
		// No medir demasiado rápido para evitar ecos fantasmas.
		osDelay(200); // 5 lecturas por segundo es más que suficiente.
	}
	/* USER CODE END UltrasonidoTask */
}

/* USER CODE BEGIN Header_GPSTask */
/**
 * @brief Function implementing the Geoposicion thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GPSTask */
void GPSTask(void *argument)
{
	/* USER CODE BEGIN GPSTask */
	GPS_Data_t local_gps_data;

	/* Infinite loop */
	for(;;)
	{
		GPS_ProcessData();
		if (gps_data_ready) {
			// Copiar datos globales a local
			memcpy(&local_gps_data, &g_gps_data, sizeof(GPS_Data_t));

			// Enviar a cola para otras tareas
			osMessageQueuePut(gpsDataQueueHandle, &local_gps_data, 0, 0);

			// Imprimir para debug
			GPS_PrintData(&local_gps_data);

			// Limpiar bandera
			gps_data_ready = 0;
		}
		osDelay(100);
	}
	/* USER CODE END GPSTask */
}

/* USER CODE BEGIN Header_TransmisionTask */
/**
 * @brief Function implementing the Transmision thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TransmisionTask */
void TransmisionTask(void *argument)
{
	/* USER CODE BEGIN TransmisionTask */

	Sensors_I2C_Handle_t i2c_data;
	GPS_Data_t gps_data;
	LoRa_t lora_module;

	// La estructura que rellenaremos y enviaremos al logger
	Datalog_Entry_t current_log_entry = {0};
	char msg[128];
	osDelay(1000); // Espera inicial
	LoRa_Init(&lora_module, &huart1);
	LoRa_Setup(&lora_module,
			"1",  // Dirección del dispositivo
			"18", // ID de Red
			"915000000"); // Banda (ej. 915 MHz)

	/* Infinite loop */
	for(;;)
	{


		// 1. Esperar por datos de los sensores I2C (cada 2 seg)
		if (osMessageQueueGet(sensorDataQueueHandle, &i2c_data, NULL, osWaitForever) == osOK)
		{
			// 2. Intentar obtener los últimos datos del GPS (no esperar)
			// (Esto vacía la cola de GPS por si hay datos nuevos)
			osMessageQueueGet(gpsDataQueueHandle, &gps_data, NULL, 0);

			// 3. Rellenar la estructura de log
			current_log_entry.timestamp = gps_data.timestamp; // O mejor, un timestamp del GPS
			current_log_entry.latitude = gps_data.latitude;
			current_log_entry.longitude = gps_data.longitude;
			current_log_entry.gps_fix = gps_data.fix_quality;
			current_log_entry.temperature = i2c_data.data.temperature;
			current_log_entry.humidity = i2c_data.data.humidity;
			current_log_entry.eco2 = i2c_data.data.eco2;
			current_log_entry.tvoc = i2c_data.data.tvoc;
			current_log_entry.light = i2c_data.data.light;

			// 4. Enviar la estructura COMPLETA a la cola de Log
			osMessageQueuePut(logQueueHandle, &current_log_entry, 0, 0);

			// 5. Formatear y enviar por LoRa (como antes)
			snprintf(msg, sizeof(msg),
					"%.1f,%.1f,%u,%u,%.2f,%.2f,%.2f,%.1f",
					i2c_data.data.temperature,
					i2c_data.data.humidity,
					i2c_data.data.eco2,
					i2c_data.data.tvoc,
					i2c_data.data.accel[0],
					i2c_data.data.accel[1],
					i2c_data.data.accel[2],
					i2c_data.data.light); // Nos falta el gps

			char cmd[160];
			snprintf(cmd, sizeof(cmd), "AT+SEND=0,%d,%s\r\n", (int)strlen(msg), msg);
			Serial_PrintString(cmd);
		}
	}

	/* USER CODE END TransmisionTask */
}

/* USER CODE BEGIN Header_SensoresTask */
/**
 * @brief Function implementing the Sensores_I2C thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SensoresTask */
void SensoresTask(void *argument)
{
	/* USER CODE BEGIN SensoresTask */
	Sensors_I2C_Handle_t hsensors = {0};


	if (Sensors_I2C_Init(&hsensors, &hi2c1)==HAL_OK)
	{
		Serial_PrintString("Sensores I2C Inicializados \n\r");
	}
	else
	{
		Serial_PrintString("No se logró inicializar los sensores \n\r");
	}


	/* Infinite loop */
	for(;;)
	{
		// 1. Leer Temperatura y Humedad
		HDC1080_Read(&hsensors);

		// 2. Enviar datos de T/H al CCS811 para compensación
		CCS811_WriteEnvData(&hsensors, hsensors.data.temperature, hsensors.data.humidity);

		// 3. Leer Calidad de Aire (ahora compensada)
		CCS811_Read(&hsensors);

		// 4. Leer el resto de sensores
		MPU6050_Read(&hsensors);
		LTR390_Read(&hsensors);

		portENTER_CRITICAL();
		g_current_lux = hsensors.data.light;
		portEXIT_CRITICAL();

		osMessageQueuePut(sensorDataQueueHandle, &hsensors, 0, 0);
		osDelay(200);

	}

	/* USER CODE END SensoresTask */
}

/* USER CODE BEGIN Header_Datos_SD */
/**
 * @brief Function implementing the logTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Datos_SD */
void Datos_SD(void *argument)
{
	/* USER CODE BEGIN Datos_SD */
	Datalog_Entry_t log_entry;
	Datalogger_t datalogger;
	char cmd[256];
	snprintf(cmd, sizeof(cmd), "Tarea SD Inicializada\r\n");
	Serial_PrintString(cmd);
	osDelay(2000);
	snprintf(cmd, sizeof(cmd), "[SD] Intentando montar...\r\n");
	Serial_PrintString(cmd);
	// 1. Inicializar el Datalogger
	// Esperamos un poco para que la SD se estabilice
	osDelay(2000);
	if (Datalogger_Init(&datalogger, "LOG_001.BIN") != HAL_OK) {
		// Error: No se pudo montar la SD.

		snprintf(cmd, sizeof(cmd), "[SD] FALLO al montar\r\n");
		Serial_PrintString(cmd);
		//osThreadTerminate(NULL); // Terminar la tarea
	}
	else
	{
		snprintf(cmd, sizeof(cmd), "[SD] Montaje EXITOSO\r\n");
		Serial_PrintString(cmd);
	}
	/* Infinite loop */
	for(;;)
	{
		log_entry.eco2 = 10;
		log_entry.gps_fix= 11;
		log_entry.humidity = 12;
		log_entry.latitude = 13;
		log_entry.light = 14;
		log_entry.longitude = 15;
		log_entry.temperature = 16;
		log_entry.timestamp = 17;
		log_entry.tvoc= 18;

		Datalogger_LogEntry(&datalogger, &log_entry);
		// 2. Esperar (bloquearse) hasta que lleguen datos a la cola de log
		/*
		if (osMessageQueueGet(logQueueHandle, &log_entry, NULL, osWaitForever) == osOK)
		{
			// 3. Escribir los datos en la SD
			/Datalogger_LogEntry(&datalogger, &log_entry);
			// La tarea se bloqueará en f_sync() (dentro de LogEntry),
			// lo cual es perfecto para una tarea de baja prioridad.
		}
		 */
		osDelay(1000);
	}
	/* USER CODE END Datos_SD */
}

/* USER CODE BEGIN Header_PatasRoverTask */
/**
 * @brief Function implementing the Servos thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PatasRoverTask */
void PatasRoverTask(void *argument)
{
	/* USER CODE BEGIN PatasRoverTask */
	Servo_Init(&hServoDiag1, &htim4, TIM_CHANNEL_1);
	Servo_Init(&hServoDiag2, &htim4, TIM_CHANNEL_2);
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END PatasRoverTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

