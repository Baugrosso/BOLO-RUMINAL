/**
 * @file    bolo_power.h
 * @brief   Bajo consumo y despertadores del BOLO_TX (STM32L432).
 *
 *  El micro pasa casi todo el tiempo en STOP2 y lo despiertan tres cosas:
 *   - el tick de 1 s del RTC (bolo_reloj.c),
 *   - el pin INT del MPU-6050 en PB1 (una muestra nueva, 5 por segundo),
 *   - el LPTIM1 (fin de los 16 ms de medicion del SHT31).
 *  Durante un contacto de radio no se entra en STOP2 (hace falta el SysTick para
 *  la ranura de 0,4 s): se espera en SLEEP y el muestreo sigue igual.
 */
#ifndef BOLO_POWER_H
#define BOLO_POWER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Configura el pin INT del MPU (PB1, EXTI1), el LPTIM1 (reloj LSE) y el STOP2.
 * Llamar una vez al arrancar, despues de MX_GPIO_Init(). */
void power_init(UART_HandleTypeDef *huart);

/* STOP2 hasta el proximo despertador. No duerme si ya hay algo pendiente. */
void power_dormir(void);

/* SLEEP hasta la proxima interrupcion (el SysTick despierta cada 1 ms) */
void power_esperar(void);

/* Espera en SLEEP hasta que pasen 'ms' desde 't0' (un valor de HAL_GetTick()) */
void power_esperar_desde(uint32_t t0, uint32_t ms);

/* Muestras del MPU avisadas por INT desde la ultima consulta (y las borra) */
uint32_t power_mpu_avisos(void);

/* Temporizador de un disparo con el LPTIM1 (sigue en STOP2). Hasta ~2 s. */
void power_lptim_iniciar(uint32_t ms);
bool power_lptim_vencio(void);          /* true una sola vez, cuando vencio */

/* Ademas, bolo_power.c reemplaza a HAL_Delay() (que en la HAL es "weak"): las
 * esperas de los drivers duermen en SLEEP en vez de dar vueltas en un bucle. */

#endif /* BOLO_POWER_H */
