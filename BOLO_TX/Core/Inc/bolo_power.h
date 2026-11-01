/**
 * @file    bolo_power.h
 * @brief   Modos de bajo consumo del nodo BOLO_TX (STM32L432).
 *          La aplicacion pide "dormi N segundos" y este modulo decide como.
 */
#ifndef BOLO_POWER_H
#define BOLO_POWER_H

#include "main.h"
#include <stdint.h>

/* Guarda los handles y deja el despertador apagado. Llamar una vez al arrancar. */
void power_init(RTC_HandleTypeDef *hrtc, UART_HandleTypeDef *huart);

/* Espera hasta que pasen 'ms' desde 't0' (un valor de HAL_GetTick()) durmiendo
 * en SLEEP: el nucleo se apaga y el SysTick lo despierta cada 1 ms para mirar la
 * hora. Se usa entre muestras del acelerometro (cada 40 ms). */
void power_esperar_desde(uint32_t t0, uint32_t ms);

/* Duerme en STOP2 durante 'segundos' y lo despierta el RTC.
 * Al volver, todo sigue configurado igual (STOP2 conserva RAM y registros). */
void power_dormir_s(uint32_t segundos);

/* Ademas, bolo_power.c reemplaza a HAL_Delay() (que en la HAL es "weak"): todas
 * las esperas de los drivers (despertar del MPU, medicion del SHT31, reset de la
 * radio) pasan a dormir en SLEEP en vez de dar vueltas en un bucle. */

#endif /* BOLO_POWER_H */
