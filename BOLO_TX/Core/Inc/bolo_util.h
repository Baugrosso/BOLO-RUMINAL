/**
 * @file    bolo_util.h
 * @brief   Utilidades de bajo nivel: CRC-32, medicion de VDD y numeros al azar.
 */
#ifndef BOLO_UTIL_H
#define BOLO_UTIL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* CRC-32 estandar (el de zlib/Ethernet: polinomio 0x04C11DB7 reflejado, inicio y
 * XOR final 0xFFFFFFFF). En el micro lo hace la unidad CRC por hardware; en Python
 * es zlib.crc32(). Se puede calcular por partes encadenando 'previo'
 * (la primera llamada con previo = 0). */
uint32_t crc32_calc(const uint8_t *datos, size_t n, uint32_t previo);

/* Prueba la unidad por hardware contra el valor de referencia. Si falla, desde
 * ahi en adelante se calcula por software (mismo resultado, mas lento). */
bool     crc32_autotest(void);

/* Tension de alimentacion [mV], medida con la referencia interna (VREFINT) y su
 * calibracion de fabrica. Prende el ADC, mide y lo deja apagado. 0 si fallo. */
uint16_t vdd_medir_mv(void);

/* Codigo de 8 bits de los metadatos: VDD = 1,80 V + codigo x 7 mV */
uint8_t  vdd_codigo(uint16_t mv);

/* Numeros al azar (xorshift32). La semilla mezcla el ID unico del micro con el
 * reloj: dos bolos que arrancan juntos no sortean lo mismo. */
void     azar_semilla(uint32_t extra);
uint32_t azar_rango(uint32_t min, uint32_t max);   /* entero en [min, max] */

#endif /* BOLO_UTIL_H */
