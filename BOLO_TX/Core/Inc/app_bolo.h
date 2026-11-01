/**
 * @file    app_bolo.h
 * @brief   Aplicacion del nodo BOLO_TX: el ciclo SENSAR -> PROCESAR -> REPORTAR -> DORMIR.
 */
#ifndef APP_BOLO_H
#define APP_BOLO_H

#include "main.h"

/* Recibe los handles que crea CubeMX: asi app_bolo no depende de donde los genera */
void app_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart, RTC_HandleTypeDef *hrtc,
              IWDG_HandleTypeDef *hiwdg, SPI_HandleTypeDef *hspi);
/* Ejecuta un ciclo completo. Se llama dentro del while(1) de main.c */
void app_run(void);

#endif /* APP_BOLO_H */
