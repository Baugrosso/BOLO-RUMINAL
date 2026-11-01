/**
 * @file    motion.c
 * @brief   Calcula media y desvio por eje y de la magnitud, y clasifica en
 *          REPOSO / MOVIMIENTO SUAVE / MOVIMIENTO FUERTE.
 */
#include "motion.h"
#include "bolo_config.h"
#include <math.h>     /* sqrtf, fmaxf */
#include <stddef.h>   /* NULL */

#define MIN_MUESTRAS_VALIDAS   (ACC_N_MUESTRAS / 2U)

#define UMBRAL_SUAVE_G    ((float)MOV_UMBRAL_SUAVE_MG  / 1000.0f)
#define UMBRAL_FUERTE_G   ((float)MOV_UMBRAL_FUERTE_MG / 1000.0f)

/* ===================== Tipos y variables privadas ===================== */
typedef struct
{
  int32_t suma;        /* Σx   */
  int64_t suma_cuad;   /* Σx²  */
} acum_t;

static acum_t   s_x, s_y, s_z;   /* ejes                        */
static acum_t   s_mag;           /* magnitud √(x² + y² + z²)    */
static uint32_t s_n;
static float    s_cuentas_por_g = 1.0f;

/* ===================== Funciones privadas ===================== */
static void acum_add(acum_t *a, int32_t v)
{
  a->suma      += v;
  a->suma_cuad += (int64_t)v * v;
}

static void acum_stats(const acum_t *a, uint32_t n, float *media_g, float *desvio_g)
{
  int64_t num = (int64_t)n * a->suma_cuad - (int64_t)a->suma * a->suma;
  if (num < 0) num = 0;

  if (media_g  != NULL) *media_g  = ((float)a->suma / (float)n) / s_cuentas_por_g;
  if (desvio_g != NULL) *desvio_g = (sqrtf((float)num) / (float)n) / s_cuentas_por_g;
}

/* ===================== Funciones publicas ===================== */
void motion_reset(float cuentas_por_g)
{
  s_x = (acum_t){0};
  s_y = (acum_t){0};
  s_z = (acum_t){0};
  s_mag = (acum_t){0};
  s_n = 0;
  s_cuentas_por_g = (cuentas_por_g > 0.0f) ? cuentas_por_g : 1.0f;
}

void motion_add(const acc_raw_t *m)
{
  if (m == NULL) return;

  acum_add(&s_x, m->x);
  acum_add(&s_y, m->y);
  acum_add(&s_z, m->z);

  /* Magnitud en cuentas. En float: x²+y²+z² puede pasar de 3.000 millones
   * y no entra en un entero de 32 bits con signo. */
  float fx = m->x, fy = m->y, fz = m->z;
  float mag = sqrtf(fx * fx + fy * fy + fz * fz);
  acum_add(&s_mag, (int32_t)(mag + 0.5f));

  s_n++;
}

void motion_compute(bolo_data_t *out)
{
  if (out == NULL) return;

  out->n_muestras = (uint16_t)s_n;

  if (s_n < MIN_MUESTRAS_VALIDAS)
  {
    out->media      = (acc_vec_t){0};
    out->desvio     = (acc_vec_t){0};
    out->desvio_max = 0.0f;
    out->desvio_mag = 0.0f;
    out->estado     = BOLO_ESTADO_DESCONOCIDO;
    return;
  }

  acum_stats(&s_x,   s_n, &out->media.x, &out->desvio.x);
  acum_stats(&s_y,   s_n, &out->media.y, &out->desvio.y);
  acum_stats(&s_z,   s_n, &out->media.z, &out->desvio.z);
  acum_stats(&s_mag, s_n, NULL,          &out->desvio_mag);

  out->desvio_max = fmaxf(out->desvio.x, fmaxf(out->desvio.y, out->desvio.z));

  if (out->desvio_mag >= UMBRAL_FUERTE_G)          /* ¿fue brusco? */
  {
    out->estado = BOLO_ESTADO_MOV_FUERTE;
  }
  else if (out->desvio_max >= UMBRAL_SUAVE_G)      /* ¿se movio?   */
  {
    out->estado = BOLO_ESTADO_MOV_SUAVE;
  }
  else
  {
    out->estado = BOLO_ESTADO_REPOSO;
  }
}
