/*
 * datalogger.h
 *
 *  Created on: Nov 10, 2025
 *      Author: joadj
 */

#ifndef INC_TRANSMISION_DATALOGGER_H_
#define INC_TRANSMISION_DATALOGGER_H_


#include "ff.h" // Header principal de FatFs
#include "stm32f4xx_hal.h"
#include "GPS.h" // Asumo que aquí está tu 'GPS_Data_t'
#include "Sensors_I2C.h" // Asumo que aquí está tu 'Sensors_I2C_Data_t'

/*
 * Estructura de nuestros datos.
 * Esta es la "fila" que se escribirá en el archivo binario.
 * Usamos __attribute__((packed)) para asegurar que no haya
 * bytes de relleno y el tamaño sea exacto.
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;     // Para saber cuándo se tomó la muestra
    float latitude;
    float longitude;
    uint16_t gps_fix;       // 0 o 1

    float temperature;
    float humidity;
    uint16_t eco2;
    uint16_t tvoc;
    float light;

    // Añade aquí más datos si es necesario

} Datalog_Entry_t;


/*
 * Handle principal del Datalogger.
 * Contiene los objetos del sistema de archivos.
 */
typedef struct {
    FATFS fs;           // Objeto del sistema de archivos FatFs
    FIL file;           // Objeto de archivo FatFs
    char filename[32];
    uint8_t is_mounted;
} Datalogger_t;


/**
 * @brief Inicializa el Datalogger.
 * Monta la tarjeta SD y abre el archivo de log.
 * @param logger Puntero al handle del logger
 * @param filename Nombre del archivo (ej. "datalog.bin")
 * @return HAL_OK si todo fue exitoso.
 */
HAL_StatusTypeDef Datalogger_Init(Datalogger_t *logger, const char *filename);

/**
 * @brief Escribe una nueva entrada de datos en el archivo de log.
 * @param logger Puntero al handle del logger
 * @param entry Puntero a la estructura de datos a escribir
 * @return HAL_OK si la escritura fue exitosa.
 */
HAL_StatusTypeDef Datalogger_LogEntry(Datalogger_t *logger, Datalog_Entry_t *entry);

/**
 * @brief Cierra el archivo de log.
 * @param logger Puntero al handle del logger
 */
void Datalogger_Close(Datalogger_t *logger);


#endif /* INC_TRANSMISION_DATALOGGER_H_ */
