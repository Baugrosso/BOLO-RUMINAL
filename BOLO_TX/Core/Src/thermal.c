/**
 * @file    thermal.c
 * @brief   Linea de base aprendida y deteccion de eventos termicos.
 *
 *  1. APRENDIZAJE: las primeras TERM_APRENDIZAJE_CICLOS mediciones se promedian
 *     para la base inicial; mientras tanto no se detecta nada.
 *  2. BASE: promedio movil exponencial; en cada ciclo se mueve 1/TERM_N_BASE de
 *     la diferencia hacia la medicion. Se guarda x256 para no perder precision.
 *  3. EVENTO: delta <= -umbral -> BAJADA (posible toma de agua);
 *             delta >= +umbral -> SUBA (SOLO DEMO, ver bolo_config.h).
 *     Mientras dura, la base se congela.
 *  4. SALIDA (segun el sentido del evento, con histeresis):
 *     BAJADA termina cuando delta > -histeresis (o pasa directo a SUBA);
 *     SUBA   termina cuando delta < +histeresis (o pasa directo a BAJADA).
 *  5. TOPE: si un evento dura mas de TERM_EVENTO_MAX_CICLOS, se corta y la base
 *     se reaprende desde la medicion actual (el entorno cambio de verdad).
 *  Al terminar un evento se informa su duracion y su delta mas extremo.
 */
#include "thermal.h"
#include "bolo_config.h"
#include <stddef.h>   /* NULL */

#define ESCALA   256L

static term_estado_t s_estado   = TERM_APRENDIENDO;
static int32_t       s_base_esc = 0;   /* base [centi-C] x ESCALA                     */
static int32_t       s_suma     = 0;   /* aprendizaje: suma de mediciones             */
static uint16_t      s_n        = 0;   /* aprendizaje: mediciones | evento: ciclos    */
static int16_t       s_extremo  = 0;   /* evento: delta mas extremo (min o max)       */

/* Division por ESCALA redondeando al entero mas cercano (tambien con negativos) */
static int32_t des_escalar(int32_t x)
{
  return (x >= 0) ? (x + ESCALA / 2) / ESCALA : -((-x + ESCALA / 2) / ESCALA);
}

static void evento_empezar(term_estado_t tipo, int32_t delta)
{
  s_estado  = tipo;
  s_n       = 1;                       /* el ciclo que lo detecta es el ciclo 1 */
  s_extremo = (int16_t)delta;
}

static void evento_terminar(bolo_data_t *d, bool por_tope)
{
  d->term_fin_tipo          = s_estado;
  d->term_fin_ciclos        = s_n;
  d->term_fin_extremo_centi = s_extremo;
  d->term_fin_por_tope      = por_tope;
  s_estado = TERM_NORMAL;
  s_n      = 0;
}

/* Sigue un evento en curso. 'dir' = -1 para BAJADA, +1 para SUBA */
static void evento_seguir(bolo_data_t *d, int32_t t, int32_t delta, int32_t dir)
{
  const int32_t umbral_contrario = (dir < 0) ?  (int32_t)TERM_UMBRAL_SUBA_CENTI
                                             : -(int32_t)TERM_UMBRAL_BAJADA_CENTI;
  const int32_t his = (int32_t)TERM_HISTERESIS_CENTI;

  if ((dir < 0 && delta >= umbral_contrario) || (dir > 0 && delta <= umbral_contrario))
  {
    evento_terminar(d, false);                                  /* se dio vuelta:     */
    evento_empezar((dir < 0) ? TERM_SUBA : TERM_BAJADA, delta); /* pasa directo       */
  }
  else if ((dir < 0 && delta > -his) || (dir > 0 && delta < his))
  {
    evento_terminar(d, false);                                  /* volvio a la base   */
  }
  else
  {
    if ((dir < 0 && delta < s_extremo) || (dir > 0 && delta > s_extremo)) s_extremo = (int16_t)delta;

    if (s_n >= TERM_EVENTO_MAX_CICLOS)
    {
      evento_terminar(d, true);                                 /* tope: el entorno   */
      s_base_esc = t * ESCALA;                                  /* cambio, nueva base */
    }
    else
    {
      s_n++;
    }
  }
}

void thermal_reset(void)
{
  s_estado   = TERM_APRENDIENDO;
  s_base_esc = 0;
  s_suma     = 0;
  s_n        = 0;
  s_extremo  = 0;
}

void thermal_update(bolo_data_t *d)
{
  if (d == NULL) return;

  /* Sin medicion valida: no se toca nada, el estado interno se conserva */
  if (!d->temp_ok)
  {
    d->term_estado = TERM_SIN_DATO;
    return;
  }

  int32_t t = d->temp_centi;

  /* ---------- 1. Aprendizaje inicial ---------- */
  if (s_estado == TERM_APRENDIENDO)
  {
    s_suma    += t;
    s_n++;
    s_base_esc = (s_suma * ESCALA) / (int32_t)s_n;   /* promedio de lo juntado */

    int32_t base = des_escalar(s_base_esc);
    d->temp_base_centi    = (int16_t)base;
    d->temp_delta_centi   = (int16_t)(t - base);
    d->term_estado        = TERM_APRENDIENDO;
    d->term_ciclos_evento = s_n;                     /* progreso: n de N */

    if (s_n >= TERM_APRENDIZAJE_CICLOS) { s_estado = TERM_NORMAL; s_n = 0; }
    return;
  }

  int32_t base  = des_escalar(s_base_esc);
  int32_t delta = t - base;

  /* ---------- 2. Maquina de estados ---------- */
  switch (s_estado)
  {
    case TERM_NORMAL:
      if      (delta <= -(int32_t)TERM_UMBRAL_BAJADA_CENTI) evento_empezar(TERM_BAJADA, delta);
      else if (delta >=  (int32_t)TERM_UMBRAL_SUBA_CENTI)   evento_empezar(TERM_SUBA,   delta);
      break;
    case TERM_BAJADA: evento_seguir(d, t, delta, -1); break;
    case TERM_SUBA:   evento_seguir(d, t, delta, +1); break;
    default:          break;
  }

  /* ---------- 3. Aprendizaje de la base: solo fuera de evento ---------- */
  if (d->term_fin_por_tope)
  {
    base  = t;                 /* se informa la base nueva */
    delta = 0;
  }
  else if (s_estado == TERM_NORMAL)
  {
    s_base_esc += (t * ESCALA - s_base_esc) / (int32_t)TERM_N_BASE;
  }

  /* ---------- 4. Resultado (base y delta usados para decidir) ---------- */
  d->temp_base_centi    = (int16_t)base;
  d->temp_delta_centi   = (int16_t)delta;
  d->term_estado        = s_estado;
  d->term_ciclos_evento = (s_estado == TERM_BAJADA || s_estado == TERM_SUBA) ? s_n : 0U;
}
