/**
 * @file    bolo_types.h
 * @brief   Tipos de datos compartidos por todos los modulos del prototipo.
 */
#ifndef BOLO_TYPES_H
#define BOLO_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 *  1. ESTADO DEL BOLO
 * ========================================================= */
typedef enum
{
  BOLO_ESTADO_DESCONOCIDO = 0,   /* sin dato valido (arranque o error) */
  BOLO_ESTADO_REPOSO,            /* = 1 */
  BOLO_ESTADO_MOV_SUAVE,         /* = 2 */
  BOLO_ESTADO_MOV_FUERTE         /* = 3 */
} bolo_estado_t;

/* Estado termico (lo decide thermal.c) */
typedef enum
{
  TERM_SIN_DATO = 0,             /* no hubo temperatura valida este ciclo      */
  TERM_APRENDIENDO,              /* juntando mediciones para la base inicial   */
  TERM_NORMAL,                   /* cerca de la linea de base                  */
  TERM_BAJADA,                   /* cayo bajo la base (posible toma de agua)   */
  TERM_SUBA                      /* subio sobre la base (SOLO DEMO, ver config) */
} term_estado_t;

/* =========================================================
 *  2. MUESTRA CRUDA DEL ACELEROMETRO (tal cual la da el sensor)
 * ========================================================= */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} acc_raw_t;                     /* 6 bytes */

/* =========================================================
 *  3. VECTOR EN UNIDADES FISICAS [g]
 * ========================================================= */
typedef struct
{
  float x;
  float y;
  float z;
} acc_vec_t;

/* =========================================================
 *  4. RESULTADO DE UN CICLO COMPLETO
 * ========================================================= */
typedef struct
{
  uint32_t      ciclo;          /* numero de ciclo desde el arranque      */
  bolo_estado_t estado;         /* REPOSO / MOVIMIENTO                    */
  acc_vec_t     media;          /* promedio de cada eje en la ventana [g] */
  acc_vec_t     desvio;         /* desvio estandar de cada eje [g]        */
  float         desvio_max;     /* el mayor de los tres desvios [g]       */
  float         desvio_mag;     /* desvio de la magnitud |a| [g]          */
  uint16_t      n_muestras;     /* muestras validas que se leyeron        */
  bool          error_sensor;   /* true si fallo la comunicacion          */

  int16_t       temp_centi;     /* temperatura en centesimas de C (2502 = 25,02 C) */
  uint16_t      hum_centi;      /* humedad interna en centesimas de %              */
  bool          temp_ok;        /* true si la lectura fue valida (CRC OK)          */

  /* Evento termico (thermal.c) */
  int16_t       temp_base_centi;    /* linea de base aprendida [centi-C]          */
  int16_t       temp_delta_centi;   /* medicion - base [centi-C]                  */
  term_estado_t term_estado;        /* SIN_DATO / NORMAL / BAJADA / SUBA          */
  uint16_t      term_ciclos_evento; /* evento: ciclos que lleva | aprendiendo: n  */
  /* Resumen del evento que TERMINO en este ciclo (term_fin_tipo = SIN_DATO: ninguno) */
  term_estado_t term_fin_tipo;          /* BAJADA o SUBA                          */
  uint16_t      term_fin_ciclos;        /* cuantos ciclos duro                    */
  int16_t       term_fin_extremo_centi; /* delta minimo (BAJADA) o maximo (SUBA)  */
  bool          term_fin_por_tope;      /* true: se corto por tope y se reaprendio la base */

  /* Diagnostico del ciclo */
  uint16_t      fallas_lectura; /* lecturas del acelerometro que fallaron          */
  uint16_t      ventana_ms;     /* duracion real de la ventana de sensado [ms]     */
  bool          ventana_cortada;/* true si se aborto (fallas seguidas o tiempo)    */
  bool          bus_recuperado; /* true si se destrabo y reinicio el bus I2C       */

  /* A futuro (se agregan aca, sin tocar el resto):
   * uint16_t bateria_mv;        <- ADC
   */
} bolo_data_t;

#endif /* BOLO_TYPES_H */
