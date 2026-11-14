/**
 * @file    bolo_types.h
 * @brief   Tipos de datos compartidos por los modulos del BOLO_TX.
 */
#ifndef BOLO_TYPES_H
#define BOLO_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* Muestra cruda del acelerometro (tal cual la da el sensor) */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} acc_raw_t;

/* Resultado de una epoca de 20 s (opcion B): sin clases, solo numeros.
 * Si la epoca es invalida (< 50 muestras), desvio_cod = 0xFF y las medias no se usan. */
typedef struct
{
  int16_t  x_mg;          /* media de cada eje [1 mg], ya limitada a 12 bits con signo */
  int16_t  y_mg;
  int16_t  z_mg;
  uint8_t  desvio_cod;    /* desvio de la magnitud, codigo log de 8 bits (ver motion.h) */
  uint16_t n_muestras;    /* muestras que entraron en la epoca                          */
} epoca_t;

/* Causa del ultimo reinicio (4 bits en los metadatos del periodo).
 * 0 = no hubo reinicio en este periodo. */
typedef enum
{
  RST_NINGUNO     = 0,
  RST_ENCENDIDO   = 1,     /* encendido o caida de tension (BOR) */
  RST_BOTON       = 2,
  RST_SOFTWARE    = 3,     /* incluye la carga desde CubeIDE     */
  RST_WATCHDOG    = 4,
  RST_HARDFAULT   = 5,
  RST_DESCONOCIDO = 15
} rst_causa_t;

#endif /* BOLO_TYPES_H */
