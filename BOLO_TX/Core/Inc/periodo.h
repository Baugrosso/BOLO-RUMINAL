/**
 * @file    periodo.h
 * @brief   El periodo en armado (6 h): junta epocas y medias de 10 min, las
 *          comprime sobre la marcha y, al cerrar, lo guarda en el almacen.
 *
 *  Contenido de un periodo (T6/T9), en el orden en que viaja:
 *    metadatos (12 B) | temperatura | humedad | desvio | X | Y | Z | CRC-32
 *  Cada lista va comprimida (compresion.h). Las listas X/Y/Z tienen solo las
 *  epocas validas (desvio != 0xFF): por eso el desvio va antes.
 *
 *  Grilla: la epoca k cubre [marca + 20 k, +20 s) y la media j de temperatura y
 *  humedad cubre [marca + 600 j, +600 s). Un periodo completo arranca en un multiplo
 *  de 6 h; uno parcial (arranque, reset, salto de hora) arranca en una epoca.
 *
 *  Respaldo: el periodo en armado vive en RAM .noinit_sys. Cada 10 min se toma
 *  una foto chica del estado con CRC; despues de un reset (watchdog, HardFault) el
 *  periodo se recupera tal como estaba en la ultima foto y se cierra como parcial.
 */
#ifndef PERIODO_H
#define PERIODO_H

#include <stdint.h>
#include <stdbool.h>
#include "bolo_types.h"

/* Metadatos que no salen del periodo en si (los pone quien lo cierra) */
typedef struct
{
  uint16_t n_periodo;
  uint8_t  vdd_cod;
  uint8_t  sin_ack;          /* contactos sin ACK (se satura en 15)        */
} periodo_cierre_t;

/* Resumen de lo que se guardo (para el informe) */
typedef struct
{
  uint16_t n_periodo;
  uint32_t marca;
  uint16_t n_epocas;
  uint16_t n_validas;
  uint8_t  n_th;
  uint16_t bytes_nucleo;     /* metadatos + temperatura + humedad          */
  uint16_t bytes_acel;       /* desvio + X + Y + Z                         */
  bool     guardado;
} periodo_info_t;

void     periodo_abrir(uint32_t marca, bool sin_hora, uint8_t causa_reset);
bool     periodo_abierto(void);
uint32_t periodo_marca(void);
uint16_t periodo_n_epocas(void);
uint8_t  periodo_n_th(void);
bool     periodo_sin_hora(void);

/* Agrega una epoca (o una invalida si e == NULL). false si el periodo esta lleno. */
bool     periodo_epoca(const epoca_t *e);

/* Una medicion del SHT31 [centesimas de C y de %RH] dentro de la media actual */
void     periodo_muestra_th(int16_t t_centi, uint16_t h_centi);

/* Cierra la media de 10 min (sin mediciones -> "sin dato") */
bool     periodo_cerrar_th(void);

/* Cuenta fallas de sensor del periodo (se saturan en 15 cada una) */
void     periodo_falla_acc(void);
void     periodo_falla_temp(void);

/* true si la media de 10 min en curso ya tiene alguna medicion */
bool     periodo_th_pendiente(void);

/* Descarta el periodo en armado sin guardarlo (por ejemplo, si no llego a tener
 * ninguna epoca) */
void     periodo_descartar(void);

/* Cierra el periodo y lo guarda en el almacen. Queda cerrado (hay que abrir otro). */
void     periodo_cerrar(const periodo_cierre_t *c, periodo_info_t *info);

/* Respaldo .noinit: foto cada 10 min y recuperacion al arrancar */
void     periodo_respaldar(void);
bool     periodo_recuperar(void);    /* true: quedo abierto el periodo de antes del reset */

#endif /* PERIODO_H */
