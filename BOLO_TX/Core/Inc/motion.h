/**
 * @file    motion.h
 * @brief   Resumen de una epoca de 20 s del acelerometro (opcion B, T5).
 *          No clasifica: por epoca da la media de X/Y/Z y el desvio de la magnitud.
 *          No accede a hardware: recibe muestras y devuelve numeros.
 *
 *  Desvio de la magnitud, codigo log de 8 bits (T6):
 *    0..254 -> sigma = 0,1 mg * R^codigo, con R = 20000^(1/254) ~ 1,0398 (pasos ~4 %)
 *              (0 = 0,1 mg o menos; 254 = 2 g o mas)
 *    255    -> epoca invalida (menos de 50 muestras): sus medias no se mandan.
 */
#ifndef MOTION_H
#define MOTION_H

#include "bolo_types.h"

void    motion_reset(void);                  /* arranca una epoca nueva           */
void    motion_add(const acc_raw_t *m);      /* suma una muestra                  */
void    motion_epoca(epoca_t *out);          /* cierra la epoca (no resetea)      */
uint16_t motion_n(void);                     /* muestras acumuladas en la epoca   */

uint8_t motion_desvio_codigo(float sigma_mg);   /* publica para poder probarla */

#endif /* MOTION_H */
