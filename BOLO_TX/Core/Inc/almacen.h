/**
 * @file    almacen.h
 * @brief   Almacen de periodos en RAM (T8). Dos zonas en la seccion .noinit:
 *            nucleo      8 KB: metadatos + temperatura + humedad (~37 dias tipico)
 *            aceleracion 40 KB: desvio + X + Y + Z             (~5 dias tipico)
 *
 *  - Un periodo se borra SOLO con ACK, y entero.
 *  - Si una zona se llena se pisa lo mas viejo de esa zona, de a un periodo (antes
 *    se compactan los huecos que dejaron los periodos ya borrados). Al nucleo le
 *    entran muchos mas periodos: la temperatura sobrevive aunque se pierda la
 *    aceleracion de ese periodo.
 *  - Cada registro lleva su CRC: despues de un reset por watchdog se revalida todo
 *    y se descarta desde el primer registro roto.
 *
 *  Registro (alineado a 4 bytes):
 *    marca u16 | n. periodo u16 | largo u16 | banderas u8 | reserva u8 | CRC-32 | datos
 *    El CRC cubre n. periodo, largo y datos (las banderas pueden cambiar).
 */
#ifndef ALMACEN_H
#define ALMACEN_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  const uint8_t *p;
  uint16_t       n;
} alm_parte_t;

typedef struct
{
  uint16_t n_periodo;
  uint16_t largo_nucleo;
  uint16_t largo_acel;        /* 0 si no hay (sin epocas o se piso) */
  bool     sin_hora;
} alm_periodo_t;

typedef struct
{
  uint32_t uso_nucleo;        /* bytes ocupados (incluye borrados sin compactar) */
  uint32_t uso_acel;
  uint16_t periodos;          /* periodos guardados sin ACK                      */
  uint16_t pisados_nucleo;    /* periodos perdidos por falta de lugar            */
  uint16_t pisados_acel;
} alm_estado_t;

void     almacen_init(void);           /* revalida lo que haya en RAM (despues de un reset) */
void     almacen_vaciar(void);

bool     almacen_guardar(uint16_t n_periodo, bool sin_hora,
                         const alm_parte_t *nuc, uint8_t n_nuc,
                         const alm_parte_t *acel, uint8_t n_acel);

uint16_t almacen_cantidad(void);
/* k = 0 es el mas nuevo */
bool     almacen_nuevo(uint16_t k, alm_periodo_t *out);

/* Lee bytes del nucleo o de la aceleracion de un periodo (ya ubicado con almacen_nuevo) */
uint16_t almacen_leer_nucleo(uint16_t n_periodo, uint16_t offset, uint8_t *dst, uint16_t n);
uint16_t almacen_leer_acel(uint16_t n_periodo, uint16_t offset, uint8_t *dst, uint16_t n);

bool     almacen_borrar(uint16_t n_periodo);   /* con ACK: los dos registros */

/* Primera hora de la gateway: suma 'delta' a la marca de tiempo de los periodos
 * cerrados antes de tener hora y les saca la bandera. */
void     almacen_reestampar(int32_t delta);

void     almacen_estado(alm_estado_t *e);

#endif /* ALMACEN_H */
