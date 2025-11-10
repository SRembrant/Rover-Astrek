/*
 * datalogger.c
 *
 *  Created on: Nov 10, 2025
 *      Author: joadj
 */

#include "datalogger.h"
#include <string.h>

// Variable global para el objeto FatFs.
// La librería de bajo nivel (sd_diskio.c) la necesita.
FATFS SDFatFS;
// Nombre del "drive" (siempre "0:" por defecto)
char SDPath[4] = "0:/";

HAL_StatusTypeDef Datalogger_Init(Datalogger_t *logger, const char *filename) {
    FRESULT res;

    // Guardar nombre del archivo
    strncpy(logger->filename, filename, sizeof(logger->filename));
    logger->is_mounted = 0;

    // 1. Montar el sistema de archivos
    // Necesitas vincular SDFatFS y SDPath a tu 'datalogger'
    // o simplemente usar la variable global SDFatFS
    res = f_mount(&logger->fs, (TCHAR const*)SDPath, 1); // 1 = montar ahora

    if (res != FR_OK) {
        // Error al montar (¿No hay tarjeta? ¿Mal formato?)
        return HAL_ERROR;
    }

    // 2. Abrir el archivo de log
    // FA_OPEN_APPEND: Abre si existe y va al final. Si no existe, lo crea.
    // FA_WRITE: Necesitamos permiso de escritura.
    res = f_open(&logger->file, logger->filename, FA_OPEN_APPEND | FA_WRITE);

    if (res != FR_OK) {
        f_mount(NULL, (TCHAR const*)SDPath, 1); // Desmontar
        return HAL_ERROR;
    }

    logger->is_mounted = 1;
    return HAL_OK;
}

HAL_StatusTypeDef Datalogger_LogEntry(Datalogger_t *logger, Datalog_Entry_t *entry) {
    if (!logger->is_mounted) {
        return HAL_ERROR; // No se ha inicializado
    }

    FRESULT res;
    UINT bytes_written = 0;

    // 1. Escribir la estructura binaria (struct) directamente en el archivo
    res = f_write(&logger->file, entry, sizeof(Datalog_Entry_t), &bytes_written);

    if (res != FR_OK || bytes_written != sizeof(Datalog_Entry_t)) {
        // Error durante la escritura
        return HAL_ERROR;
    }

    // 2. Sincronizar el archivo (¡CRÍTICO!)
    // Esto fuerza al sistema a escribir el buffer en la tarjeta SD física.
    // Si no haces esto y quitas la energía, perderás los últimos datos.
    // Es una operación lenta, pero perfecta para una tarea de baja prioridad.
    res = f_sync(&logger->file);

    if (res != FR_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void Datalogger_Close(Datalogger_t *logger) {
    if (logger->is_mounted) {
        f_close(&logger->file);
        f_mount(NULL, (TCHAR const*)SDPath, 1); // Desmontar
        logger->is_mounted = 0;
    }
}
