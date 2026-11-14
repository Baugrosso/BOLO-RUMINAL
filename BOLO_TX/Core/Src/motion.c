/**
 * @file    motion.c
 * @brief   Media por eje y desvio de la magnitud de una epoca de 20 s.
 *          Los acumuladores se reinician cada 20 s (motion_reset).
 */
#include "motion.h"
#include "bolo_config.h"
#include "sensor_acc.h"
#include <math.h>     /* sqrtf, logf, lroundf */
#include <stddef.h>   /* NULL */

/* La magnitud se acumula en 1/16 de cuenta: mejora la resolucion del desvio sin
 * salirse de los enteros (16 * 28378 cuentas maximas = 454 048 -> int32). */
#define MAG_ESCALA        16

/* Codigo log del desvio: 0,1 mg .. 2000 mg en 254 pasos */
#define DESV_MIN_MG       0.1f
#define DESV_PASOS        254.0f
#define DESV_RANGO        20000.0f      /* 2000 mg / 0,1 mg */

/* ===================== Variables privadas ===================== */
static int32_t  s_sx, s_sy, s_sz;     /* suma de cada eje [cuentas]          */
static int64_t  s_sm, s_smm;          /* suma y suma de cuadrados de |a|*16  */
static uint16_t s_n;

/* ===================== Funciones privadas ===================== */
static int16_t media_mg(int32_t suma, uint16_t n)
{
  /* cuentas -> mg: x * 1000 / 16384, redondeado al entero mas cercano */
  float mg = ((float)suma / (float)n) * (1000.0f / (float)ACC_CUENTAS_POR_G);
  long  r  = lroundf(mg);
  if (r > FMT_ACC_MAX_MG) r = FMT_ACC_MAX_MG;
  if (r < FMT_ACC_MIN_MG) r = FMT_ACC_MIN_MG;
  return (int16_t)r;
}

/* ===================== Funciones publicas ===================== */
uint8_t motion_desvio_codigo(float sigma_mg)
{
  if (!(sigma_mg > DESV_MIN_MG)) return 0U;          /* tambien atrapa NaN */
  float c = logf(sigma_mg / DESV_MIN_MG) * (DESV_PASOS / logf(DESV_RANGO));
  if (c >= DESV_PASOS) return (uint8_t)DESV_PASOS;
  return (uint8_t)lroundf(c);
}

void motion_reset(void)
{
  s_sx = s_sy = s_sz = 0;
  s_sm = s_smm = 0;
  s_n = 0;
}

void motion_add(const acc_raw_t *m)
{
  if (m == NULL || s_n == 0xFFFFU) return;

  s_sx += m->x;
  s_sy += m->y;
  s_sz += m->z;

  /* Magnitud en float: x²+y²+z² puede pasar de 2^31 */
  float fx = m->x, fy = m->y, fz = m->z;
  int32_t mag = (int32_t)(sqrtf(fx * fx + fy * fy + fz * fz) * (float)MAG_ESCALA + 0.5f);
  s_sm  += mag;
  s_smm += (int64_t)mag * mag;
  s_n++;
}

uint16_t motion_n(void)
{
  return s_n;
}

void motion_epoca(epoca_t *out)
{
  if (out == NULL) return;

  out->n_muestras = s_n;

  if (s_n < ACC_MIN_VALIDAS)
  {
    out->x_mg = out->y_mg = out->z_mg = 0;
    out->desvio_cod = FMT_DESVIO_INVALIDO;
    return;
  }

  out->x_mg = media_mg(s_sx, s_n);
  out->y_mg = media_mg(s_sy, s_n);
  out->z_mg = media_mg(s_sz, s_n);

  /* sigma = sqrt(n*Σx² - (Σx)²) / n, en enteros de 64 bits (sin cancelacion) */
  int64_t num = (int64_t)s_n * s_smm - s_sm * s_sm;
  if (num < 0) num = 0;
  float sigma_cuentas = (sqrtf((float)num) / (float)s_n) / (float)MAG_ESCALA;
  float sigma_mg      = sigma_cuentas * (1000.0f / (float)ACC_CUENTAS_POR_G);

  out->desvio_cod = motion_desvio_codigo(sigma_mg);
}
