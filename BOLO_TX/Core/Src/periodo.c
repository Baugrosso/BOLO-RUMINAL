/**
 * @file    periodo.c
 * @brief   Armado del periodo: listas comprimidas, respaldo y cierre (ver periodo.h).
 */
#include "periodo.h"
#include "bolo_config.h"
#include "compresion.h"
#include "almacen.h"
#include "bolo_packet.h"
#include "bolo_util.h"
#include <string.h>
#include <stddef.h>

#ifdef BOLO_HOST
  #define NOINIT_SYS
#else
  #define NOINIT_SYS  __attribute__((section(".noinit_sys")))
#endif

/* ===================== Listas ===================== */
enum { L_TEMP = 0, L_HUM, L_DESV, L_X, L_Y, L_Z, N_LISTAS };

#define CAP_TEMP   CMP_BYTES_MAX(MEDIAS_TH_POR_PERIODO, 16U)
#define CAP_HUM    CMP_BYTES_MAX(MEDIAS_TH_POR_PERIODO, 8U)
#define CAP_DESV   CMP_BYTES_MAX(EPOCAS_POR_PERIODO, 8U)
#define CAP_EJE    CMP_BYTES_MAX(EPOCAS_POR_PERIODO, 12U)

/* Estado chico: es lo que se copia en cada foto */
typedef struct
{
  uint32_t marca;
  uint16_t n_epocas;
  uint16_t n_validas;
  uint8_t  n_th;
  uint8_t  abierto;
  uint8_t  sin_hora;
  uint8_t  causa_reset;
  uint8_t  fallas_acc;
  uint8_t  fallas_temp;
  uint8_t  th_n;                 /* mediciones en la media actual */
  uint8_t  rsv;
  int32_t  th_suma_t;            /* [centi C]   */
  uint32_t th_suma_h;            /* [centi %RH] */
  cmp_t    lista[N_LISTAS];
} per_estado_t;

typedef struct
{
  per_estado_t e;
  uint8_t b_temp[CAP_TEMP];
  uint8_t b_hum[CAP_HUM];
  uint8_t b_desv[CAP_DESV];
  uint8_t b_x[CAP_EJE];
  uint8_t b_y[CAP_EJE];
  uint8_t b_z[CAP_EJE];
} per_t;

/* Foto: estado + el ultimo byte a medio llenar de cada lista. El resto de los
 * bytes ya escritos no cambia mas (las listas solo crecen), asi que se cubren con
 * el CRC sin copiarlos. */
#define FOTO_MARCA   0x50455231U     /* "PER1" */
typedef struct
{
  uint32_t     marca;
  uint32_t     tam;                  /* sizeof(per_t): detecta un cambio de firmware */
  per_estado_t e;
  uint8_t      parcial[N_LISTAS];
  uint8_t      rsv[2];
  uint32_t     crc;
} per_foto_t;

static per_t      s_per  NOINIT_SYS;
static per_foto_t s_foto NOINIT_SYS;

static uint8_t *buf_de(uint8_t l)
{
  switch (l)
  {
    case L_TEMP: return s_per.b_temp;
    case L_HUM:  return s_per.b_hum;
    case L_DESV: return s_per.b_desv;
    case L_X:    return s_per.b_x;
    case L_Y:    return s_per.b_y;
    default:     return s_per.b_z;
  }
}

static uint16_t cap_de(uint8_t l)
{
  switch (l)
  {
    case L_TEMP: return (uint16_t)CAP_TEMP;
    case L_HUM:  return (uint16_t)CAP_HUM;
    case L_DESV: return (uint16_t)CAP_DESV;
    default:     return (uint16_t)CAP_EJE;
  }
}

static void sumar_sat(uint8_t *c)
{
  if (*c < 15U) (*c)++;
}

/* ===================== Abrir / consultar ===================== */
void periodo_abrir(uint32_t marca, bool sin_hora, uint8_t causa_reset)
{
  memset(&s_per.e, 0, sizeof(s_per.e));
  s_per.e.marca       = marca;
  s_per.e.abierto     = 1U;
  s_per.e.sin_hora    = sin_hora ? 1U : 0U;
  s_per.e.causa_reset = causa_reset;

  cmp_iniciar(&s_per.e.lista[L_TEMP], s_per.b_temp, cap_de(L_TEMP), 16U, true);
  cmp_iniciar(&s_per.e.lista[L_HUM],  s_per.b_hum,  cap_de(L_HUM),   8U, false);
  cmp_iniciar(&s_per.e.lista[L_DESV], s_per.b_desv, cap_de(L_DESV),  8U, false);
  cmp_iniciar(&s_per.e.lista[L_X],    s_per.b_x,    cap_de(L_X),    12U, true);
  cmp_iniciar(&s_per.e.lista[L_Y],    s_per.b_y,    cap_de(L_Y),    12U, true);
  cmp_iniciar(&s_per.e.lista[L_Z],    s_per.b_z,    cap_de(L_Z),    12U, true);

  periodo_respaldar();
}

bool     periodo_abierto(void)   { return s_per.e.abierto != 0U; }
uint32_t periodo_marca(void)     { return s_per.e.marca; }
uint16_t periodo_n_epocas(void)  { return s_per.e.n_epocas; }
uint8_t  periodo_n_th(void)      { return s_per.e.n_th; }
bool     periodo_sin_hora(void)  { return s_per.e.sin_hora != 0U; }

void periodo_falla_acc(void)  { sumar_sat(&s_per.e.fallas_acc); }
void periodo_falla_temp(void) { sumar_sat(&s_per.e.fallas_temp); }

/* ===================== Agregar datos ===================== */
bool periodo_epoca(const epoca_t *e)
{
  if (!s_per.e.abierto || s_per.e.n_epocas >= EPOCAS_POR_PERIODO) return false;

  uint8_t desv = (e != NULL) ? e->desvio_cod : FMT_DESVIO_INVALIDO;

  (void)cmp_agregar(&s_per.e.lista[L_DESV], s_per.b_desv, desv);
  if (desv != FMT_DESVIO_INVALIDO)
  {
    (void)cmp_agregar(&s_per.e.lista[L_X], s_per.b_x, e->x_mg);
    (void)cmp_agregar(&s_per.e.lista[L_Y], s_per.b_y, e->y_mg);
    (void)cmp_agregar(&s_per.e.lista[L_Z], s_per.b_z, e->z_mg);
    s_per.e.n_validas++;
  }
  s_per.e.n_epocas++;
  return true;
}

void periodo_muestra_th(int16_t t_centi, uint16_t h_centi)
{
  if (!s_per.e.abierto || s_per.e.th_n == 0xFFU) return;
  s_per.e.th_suma_t += t_centi;
  s_per.e.th_suma_h += h_centi;
  s_per.e.th_n++;
}

/* Division entera redondeada al mas cercano (tambien con negativos) */
static int32_t div_redondeo(int32_t a, int32_t b)
{
  return (a >= 0) ? (a + b / 2) / b : -((-a + b / 2) / b);
}

bool periodo_cerrar_th(void)
{
  if (!s_per.e.abierto || s_per.e.n_th >= MEDIAS_TH_POR_PERIODO) return false;

  int32_t t = FMT_TEMP_SIN_DATO;
  int32_t h = FMT_HUM_SIN_DATO;

  if (s_per.e.th_n > 0U)
  {
    t = div_redondeo(s_per.e.th_suma_t, s_per.e.th_n);              /* 0,01 C           */
    if (t <= FMT_TEMP_SIN_DATO) t = FMT_TEMP_SIN_DATO + 1;          /* no confundir     */
    uint32_t hc = (s_per.e.th_suma_h + s_per.e.th_n / 2U) / s_per.e.th_n;   /* centi %RH */
    h = (int32_t)((hc + 25U) / 50U);                                /* pasos de 0,5 %RH */
    if (h > 200) h = 200;
  }

  (void)cmp_agregar(&s_per.e.lista[L_TEMP], s_per.b_temp, t);
  (void)cmp_agregar(&s_per.e.lista[L_HUM],  s_per.b_hum,  h);
  s_per.e.n_th++;
  s_per.e.th_suma_t = 0;
  s_per.e.th_suma_h = 0U;
  s_per.e.th_n      = 0U;
  return true;
}

bool periodo_th_pendiente(void)
{
  return s_per.e.th_n > 0U;
}

void periodo_descartar(void)
{
  s_per.e.abierto = 0U;
  s_foto.marca    = 0U;
}

/* ===================== Cerrar ===================== */
void periodo_cerrar(const periodo_cierre_t *c, periodo_info_t *info)
{
  uint8_t        meta[PKT_META_LARGO];
  uint16_t       largo[N_LISTAS];
  periodo_info_t vacio;
  periodo_info_t *inf = (info != NULL) ? info : &vacio;

  memset(inf, 0, sizeof(*inf));
  if (!s_per.e.abierto) return;

  for (uint8_t l = 0; l < N_LISTAS; l++) largo[l] = cmp_cerrar(&s_per.e.lista[l], buf_de(l));

  /* Metadatos: las cantidades salen de las listas (lo que realmente se escribio) */
  pkt_meta_t m = {0};
  m.version     = FMT_VERSION;
  m.n_epocas    = s_per.e.lista[L_DESV].n;
  m.n_periodo   = c->n_periodo;
  m.marca       = s_per.e.marca;
  m.n_th        = (uint8_t)s_per.e.lista[L_TEMP].n;
  m.vdd_cod     = c->vdd_cod;
  m.causa_reset = s_per.e.causa_reset;
  m.sin_ack     = (c->sin_ack > 15U) ? 15U : c->sin_ack;
  m.fallas_acc  = s_per.e.fallas_acc;
  m.fallas_temp = s_per.e.fallas_temp;
  pkt_meta_armar(meta, &m);

  alm_parte_t nuc[3] = {
    { meta,          PKT_META_LARGO },
    { s_per.b_temp,  largo[L_TEMP]  },
    { s_per.b_hum,   largo[L_HUM]   },
  };
  alm_parte_t acel[4] = {
    { s_per.b_desv, largo[L_DESV] },
    { s_per.b_x,    largo[L_X]    },
    { s_per.b_y,    largo[L_Y]    },
    { s_per.b_z,    largo[L_Z]    },
  };

  inf->n_periodo    = c->n_periodo;
  inf->marca        = s_per.e.marca;
  inf->n_epocas     = m.n_epocas;
  inf->n_validas    = s_per.e.lista[L_X].n;
  inf->n_th         = m.n_th;
  inf->bytes_nucleo = (uint16_t)(PKT_META_LARGO + largo[L_TEMP] + largo[L_HUM]);
  inf->bytes_acel   = (uint16_t)(largo[L_DESV] + largo[L_X] + largo[L_Y] + largo[L_Z]);

  /* Sin epocas no se guarda registro de aceleracion */
  inf->guardado = almacen_guardar(c->n_periodo, s_per.e.sin_hora != 0U,
                                  nuc, 3U, acel, (m.n_epocas > 0U) ? 4U : 0U);

  s_per.e.abierto = 0U;
  s_foto.marca    = 0U;            /* la foto ya no vale: el periodo esta guardado */
}

/* ===================== Respaldo ===================== */
static uint32_t crc_foto(const per_foto_t *f)
{
  uint32_t crc = crc32_calc((const uint8_t *)f, offsetof(per_foto_t, crc), 0U);
  for (uint8_t l = 0; l < N_LISTAS; l++)
  {
    uint32_t completos = f->e.lista[l].bits / 8U;            /* bytes que ya no cambian */
    if (completos > cap_de(l)) completos = cap_de(l);
    crc = crc32_calc(buf_de(l), completos, crc);
  }
  return crc;
}

void periodo_respaldar(void)
{
  if (!s_per.e.abierto) { s_foto.marca = 0U; return; }

  s_foto.marca = FOTO_MARCA;
  s_foto.tam   = (uint32_t)sizeof(per_t);
  s_foto.e     = s_per.e;
  s_foto.rsv[0] = s_foto.rsv[1] = 0U;
  for (uint8_t l = 0; l < N_LISTAS; l++)
  {
    uint32_t pos = s_per.e.lista[l].bits / 8U;
    s_foto.parcial[l] = (pos < cap_de(l)) ? buf_de(l)[pos] : 0U;
  }
  s_foto.crc = crc_foto(&s_foto);
}

bool periodo_recuperar(void)
{
  if (s_foto.marca != FOTO_MARCA || s_foto.tam != (uint32_t)sizeof(per_t) ||
      !s_foto.e.abierto || s_foto.crc != crc_foto(&s_foto))
  {
    s_foto.marca    = 0U;
    s_per.e.abierto = 0U;
    return false;
  }

  /* Se vuelve al estado de la foto: lo escrito despues se borra */
  s_per.e = s_foto.e;
  for (uint8_t l = 0; l < N_LISTAS; l++)
  {
    uint8_t *b   = buf_de(l);
    uint32_t pos = s_per.e.lista[l].bits / 8U;
    uint16_t cap = cap_de(l);
    if (pos < cap)
    {
      memset(&b[pos], 0, cap - pos);
      if ((s_per.e.lista[l].bits & 7U) != 0U) b[pos] = s_foto.parcial[l];
    }
  }
  return true;
}
