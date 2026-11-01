/**
 * @file    thermal.h
 * @brief   Deteccion de eventos termicos (BAJADA / SUBA) respecto de una linea
 *          de base que se aprende sola. No accede a hardware ni sabe que chip
 *          mide: solo recibe la temperatura del ciclo y devuelve un resultado.
 */
#ifndef THERMAL_H
#define THERMAL_H

#include "bolo_types.h"

void thermal_reset(void);               /* olvida la base (se aprende de nuevo)      */
void thermal_update(bolo_data_t *d);    /* procesa la temperatura del ciclo en curso */

#endif /* THERMAL_H */
