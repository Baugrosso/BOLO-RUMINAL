/**
 * @file    bolo_reloj.h
 * @brief   Hora del bolo (UNIX, en segundos) y estado que sobrevive a un reset.
 *
 *  - La hora la lleva el calendario del RTC (LSE): sigue contando en Stop 2 y a
 *    traves de un reset por watchdog. Cada ACK la ajusta a la hora de la gateway.
 *  - Un "tick" de 1 s (wake-up timer del RTC) despierta al micro: marca la grilla
 *    (epocas de 20 s, temperatura cada 60 s, periodos de 6 h) y refresca el IWDG.
 *  - Lo que tiene que sobrevivir a un reset (ranura, numero de periodo, contacto,
 *    contactos sin ACK, potencia) vive en registros de backup del RTC.
 */
#ifndef BOLO_RELOJ_H
#define BOLO_RELOJ_H

#include <stdint.h>
#include <stdbool.h>

#ifndef BOLO_HOST
#include "main.h"
void     reloj_init(RTC_HandleTypeDef *hrtc);   /* lee el calendario y arranca el tick de 1 s */
#endif

uint32_t reloj_ahora(void);                     /* hora UNIX actual [s]                       */
int32_t  reloj_fijar(uint32_t t_unix);            /* ajusta la hora; devuelve nuevo - anterior  */
bool     reloj_tick(void);                      /* true si hubo tick desde la ultima consulta */
bool     reloj_tick_pendiente(void);            /* igual, pero sin borrarlo (para dormir)      */

/* Conversion calendario <-> UNIX (anio 2000-2099), publicas para poder probarlas */
uint32_t reloj_a_unix(uint16_t anio, uint8_t mes, uint8_t dia,
                      uint8_t hora, uint8_t min, uint8_t seg);
void     reloj_de_unix(uint32_t t_unix, uint16_t *anio, uint8_t *mes, uint8_t *dia,
                       uint8_t *hora, uint8_t *min, uint8_t *seg);

/* ---------- Estado persistente (registros de backup) ---------- */
typedef struct
{
  bool     hora_sincronizada;   /* alguna vez llego la hora de la gateway */
  bool     ranura_valida;       /* la gateway ya asigno ranura            */
  uint8_t  ranura;              /* 0..179                                 */
  int8_t   potencia_dbm;        /* potencia de TX (comando 2 del ACK)     */
  uint16_t n_periodo;           /* numero del proximo periodo a cerrar    */
  uint8_t  contacto;            /* contador de contactos (8 bits)         */
  uint8_t  sin_ack;             /* contactos seguidos sin ACK             */
} bolo_persist_t;

void persist_leer(bolo_persist_t *p);           /* valores por defecto si no hay nada guardado */
void persist_guardar(const bolo_persist_t *p);

#endif /* BOLO_RELOJ_H */
