/**
 * @file    compresion.h
 * @brief   Compresion sin perdida de una lista de valores (T7), sobre la marcha.
 *
 *  Formato de una lista de n valores de W bits (flujo de bits, MSB primero):
 *   - n = 0: no ocupa nada.
 *   - Primer valor: entero, W bits (complemento a 2 si la variable tiene signo).
 *   - Los n-1 restantes, en bloques de 32 (el ultimo puede ser mas corto):
 *       d = v[i] - v[i-1];  u = zigzag(d) = (d << 1) ^ (d >> 31)
 *       etiqueta de 4 bits por bloque:
 *         0..13 -> Rice con parametro k: por cada u, (u >> k) bits en 1, un 0, y
 *                  los k bits bajos de u
 *         14    -> sin comprimir: cada VALOR (no la diferencia) en W bits
 *         15    -> todas las diferencias del bloque son cero (no hay mas bits)
 *       El bolo elige, al cerrar cada bloque, la opcion que ocupa menos bits.
 *   - La lista termina rellenada con ceros hasta el byte (la siguiente arranca en
 *     un byte nuevo).
 *  Peor caso = sin comprimir + 4 bits por bloque (CMP_BYTES_MAX).
 *
 *  El estado (cmp_t) no guarda punteros: el buffer se pasa en cada llamada. Asi
 *  todo el periodo en armado puede vivir en la RAM .noinit y sobrevivir a un reset.
 */
#ifndef COMPRESION_H
#define COMPRESION_H

#include <stdint.h>
#include <stdbool.h>
#include "bolo_config.h"

/* Bytes maximos de una lista de n valores de w bits */
#define CMP_BYTES_MAX(n, w)                                                        \
  (((n) == 0U) ? 0U :                                                              \
   (((w) + ((((n) - 1U) + CMP_BLOQUE - 1U) / CMP_BLOQUE) * 4U + ((n) - 1U) * (w)   \
     + 7U) / 8U))

#define CMP_ETQ_SIN_COMPRIMIR   14U
#define CMP_ETQ_CEROS           15U
#define CMP_K_MAX               13U

typedef struct
{
  uint32_t bits;              /* bits escritos                                   */
  uint16_t cap;               /* capacidad del buffer [bytes]                    */
  uint16_t n;                 /* valores agregados                               */
  uint8_t  ancho;             /* W                                               */
  uint8_t  con_signo;         /* 1 = complemento a 2                             */
  uint8_t  en_bloque;         /* valores esperando en el bloque actual           */
  uint8_t  desborde;          /* 1 = no entro algo (no deberia pasar nunca)      */
  int32_t  previo;            /* ultimo valor agregado                           */
  int32_t  base_bloque;       /* valor anterior al primero del bloque actual     */
  int32_t  bloque[CMP_BLOQUE];
} cmp_t;

/* Arranca una lista vacia y pone el buffer en cero */
void     cmp_iniciar(cmp_t *c, uint8_t *buf, uint16_t cap, uint8_t ancho, bool con_signo);

/* Agrega un valor (ya limitado a W bits). false si no entra. */
bool     cmp_agregar(cmp_t *c, uint8_t *buf, int32_t v);

/* Cierra el ultimo bloque y rellena hasta el byte. Devuelve los bytes de la lista.
 * Despues de cerrar no se agrega mas. */
uint16_t cmp_cerrar(cmp_t *c, uint8_t *buf);

#endif /* COMPRESION_H */
