/**
 * @file    motion.h
 * @brief   Deteccion de reposo / movimiento a partir de muestras del acelerometro.
 *          No accede a hardware: solo recibe muestras y devuelve un resultado.
 */
#ifndef MOTION_H
#define MOTION_H

#include "bolo_types.h"

void motion_reset(float cuentas_por_g);    /* arranca una ventana nueva          */
void motion_add(const acc_raw_t *m);       /* suma una muestra a la ventana      */
void motion_compute(bolo_data_t *out);     /* cierra la ventana y decide estado  */

#endif /* MOTION_H */
