/**
 * @file    app_gw.h
 * @brief   Aplicacion del gateway (nodo RX): recibe por LoRa y muestra en PuTTY.
 */
#ifndef APP_GW_H
#define APP_GW_H

#include "main.h"

/* Recibe los handles que crea CubeMX (igual que en el TX) */
void app_init(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);

/* Se llama dentro del while(1) de main.c */
void app_run(void);

#endif /* APP_GW_H */
