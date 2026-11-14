/**
 * @file    salto.h
 * @brief   Salto de frecuencia (T10). Igual en el bolo y en la gateway.
 *
 *  64 canales de 200 kHz: f = 915,2 MHz + 0,2 MHz x n (n = 0..63).
 *  Canal de una ranura = tabla[(32 x contacto + pos) mod 64], donde pos cuenta las
 *  ranuras de 0,4 s del contacto desde el primer fragmento: la rafaga ocupa
 *  pos = 0..N-1, el ACK la siguiente, cada reenvio la siguiente, etc.
 *  Un canal se repite cada 64 ranuras = 25,6 s (> 20 s que pide ENACOM).
 *  El hola y el te oigo van en un canal fijo de encuentro.
 *  (Pendiente para homologacion: confirmar que ENACOM acepta el canal fijo.)
 */
#ifndef SALTO_H
#define SALTO_H

#include <stdint.h>

#define SALTO_N_CANALES       64U
#define SALTO_BASE_HZ         915200000UL
#define SALTO_PASO_HZ         200000UL
#define SALTO_CANAL_ENCUENTRO 0U          /* 915,2 MHz */

uint8_t  salto_canal(uint8_t contacto, uint16_t pos);
uint32_t salto_hz(uint8_t canal);

#endif /* SALTO_H */
