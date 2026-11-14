/**
 * @file    contacto.c
 * @brief   Motor del contacto: plan, hola/te oigo, rafaga con salto, ACK y reenvios.
 */
#include "contacto.h"
#include "bolo_config.h"
#include "bolo_packet.h"
#include "almacen.h"
#include "bolo_util.h"
#include <string.h>

/* ===================== Vista de un periodo tal como viaja =====================
 *   [nucleo: metadatos + temperatura + humedad][aceleracion][CRC-32 big-endian]
 * Si la aceleracion de ese periodo se piso por falta de lugar, la cantidad de
 * epocas de los metadatos viaja en 0 (no hay listas de aceleracion). */
uint32_t contacto_largo_periodo(uint16_t largo_nucleo, uint16_t largo_acel)
{
  return (uint32_t)largo_nucleo + largo_acel + PKT_CRC_LARGO;
}

static uint16_t leer_sin_crc(uint16_t np, uint16_t ln, uint16_t la,
                             uint32_t offset, uint8_t *dst, uint16_t n)
{
  uint16_t hecho = 0U;

  while (hecho < n)
  {
    uint32_t o = offset + hecho;
    uint16_t k;

    if (o < ln)
    {
      k = almacen_leer_nucleo(np, (uint16_t)o, &dst[hecho], (uint16_t)(n - hecho));
      if (la == 0U)
      {
        /* sin aceleracion: cant. de epocas = 0 (se conserva la version) */
        for (uint16_t i = 0; i < k; i++)
        {
          if (o + i == 0U) dst[hecho + i] &= 0xF0U;
          if (o + i == 1U) dst[hecho + i]  = 0x00U;
        }
      }
    }
    else if (o < (uint32_t)ln + la)
    {
      k = almacen_leer_acel(np, (uint16_t)(o - ln), &dst[hecho], (uint16_t)(n - hecho));
    }
    else
    {
      break;
    }
    if (k == 0U) break;                      /* el periodo ya no esta (no deberia) */
    hecho += k;
  }
  return hecho;
}

uint32_t contacto_crc_periodo(uint16_t n_periodo, uint16_t largo_nucleo, uint16_t largo_acel)
{
  uint8_t  tmp[PKT_FRAG_DATOS];
  uint32_t crc   = 0U;
  uint32_t total = (uint32_t)largo_nucleo + largo_acel;

  for (uint32_t o = 0U; o < total; o += sizeof(tmp))
  {
    uint16_t n = (uint16_t)(((total - o) < sizeof(tmp)) ? (total - o) : sizeof(tmp));
    uint16_t k = leer_sin_crc(n_periodo, largo_nucleo, largo_acel, o, tmp, n);
    if (k < n) memset(&tmp[k], 0, n - k);
    crc = crc32_calc(tmp, n, crc);
  }
  return crc;
}

uint16_t contacto_leer_periodo(uint16_t n_periodo, uint16_t largo_nucleo, uint16_t largo_acel,
                               uint32_t crc, uint32_t offset, uint8_t *dst, uint16_t n)
{
  uint32_t datos = (uint32_t)largo_nucleo + largo_acel;
  uint16_t hecho = 0U;

  if (offset < datos)
  {
    uint16_t pedir = (uint16_t)(((datos - offset) < n) ? (datos - offset) : n);
    uint16_t k = leer_sin_crc(n_periodo, largo_nucleo, largo_acel, offset, dst, pedir);
    if (k < pedir) memset(&dst[k], 0, pedir - k);
    hecho = pedir;
  }
  while (hecho < n && (offset + hecho) < datos + PKT_CRC_LARGO)
  {
    uint32_t i = offset + hecho - datos;               /* 0..3 */
    dst[hecho++] = (uint8_t)(crc >> (24U - 8U * i));
  }
  return hecho;
}

#ifndef BOLO_HOST
/* ===================== Motor del contacto (solo en el micro) ===================== */
#include "main.h"
#include "radio.h"
#include "salto.h"
#include "bolo_power.h"

typedef struct
{
  uint16_t n_periodo;
  uint16_t largo_nucleo;
  uint16_t largo_acel;
  uint32_t largo;           /* bytes en el aire, con CRC */
  uint32_t crc;
  uint16_t frag0;           /* primer fragmento del periodo en la rafaga */
  uint16_t nfrag;
} plan_per_t;

typedef struct
{
  plan_per_t per[CONT_MAX_PERIODOS];
  uint8_t    n_per;
  uint16_t   n_frag;
  uint8_t    nodo;
  uint8_t    contacto;
  uint32_t   t0;            /* inicio de la ranura 0 [HAL_GetTick] */
  void     (*servicio)(void);
} ctx_t;

static ctx_t s_ctx;

/* ---------- Plan: del periodo mas nuevo al mas viejo ---------- */
static void armar_plan(ctx_t *c)
{
  alm_periodo_t p;

  c->n_per  = 0U;
  c->n_frag = 0U;

  for (uint16_t k = 0; c->n_per < CONT_MAX_PERIODOS && almacen_nuevo(k, &p); k++)
  {
    uint32_t largo = contacto_largo_periodo(p.largo_nucleo, p.largo_acel);
    uint16_t nfrag = (uint16_t)((largo + PKT_FRAG_DATOS - 1U) / PKT_FRAG_DATOS);
    if ((uint32_t)c->n_frag + nfrag > CONT_MAX_FRAG) break;

    plan_per_t *pp   = &c->per[c->n_per++];
    pp->n_periodo    = p.n_periodo;
    pp->largo_nucleo = p.largo_nucleo;
    pp->largo_acel   = p.largo_acel;
    pp->largo        = largo;
    pp->crc          = contacto_crc_periodo(p.n_periodo, p.largo_nucleo, p.largo_acel);
    pp->frag0        = c->n_frag;
    pp->nfrag        = nfrag;
    c->n_frag        = (uint16_t)(c->n_frag + nfrag);

    if (c->servicio != NULL) c->servicio();      /* el CRC tarda unos ms: no perder muestras */
  }
}

/* Arma el fragmento 'indice' de la rafaga; devuelve el largo */
static uint8_t armar_fragmento(const ctx_t *c, uint16_t indice, bool ultimo, uint8_t *out)
{
  for (uint8_t i = 0; i < c->n_per; i++)
  {
    const plan_per_t *pp = &c->per[i];
    if (indice < pp->frag0 || indice >= pp->frag0 + pp->nfrag) continue;

    uint32_t offset = (uint32_t)(indice - pp->frag0) * PKT_FRAG_DATOS;
    uint16_t n = (uint16_t)(((pp->largo - offset) < PKT_FRAG_DATOS) ? (pp->largo - offset) : PKT_FRAG_DATOS);

    pkt_frag_cab(out, c->nodo, c->contacto, indice, ultimo);
    (void)contacto_leer_periodo(pp->n_periodo, pp->largo_nucleo, pp->largo_acel, pp->crc,
                                offset, &out[PKT_FRAG_CAB], n);
    return (uint8_t)(PKT_FRAG_CAB + n);
  }
  return 0U;
}

/* ---------- Esperas (con el muestreo andando) ---------- */
static void esperar_hasta(const ctx_t *c, uint32_t t)
{
  while ((int32_t)(t - HAL_GetTick()) > 0)
  {
    if (c->servicio != NULL) c->servicio();
    power_esperar();
  }
}

static radio_ev_t esperar_evento(const ctx_t *c, uint32_t tope_ms, radio_rx_t *pkt)
{
  uint32_t t0 = HAL_GetTick();
  radio_ev_t ev;

  while ((ev = radio_evento(pkt)) == RADIO_EV_NINGUNO)
  {
    if ((HAL_GetTick() - t0) > tope_ms) { radio_abortar(); return RADIO_EV_TIMEOUT; }
    if (c->servicio != NULL) c->servicio();
    power_esperar();
  }
  return ev;
}

static uint32_t inicio_ranura(const ctx_t *c, uint16_t pos)
{
  return c->t0 + (uint32_t)pos * CONT_T_RANURA_MS;
}

/* Manda el fragmento 'indice' en la ranura 'pos'. false si la radio fallo. */
static bool enviar(const ctx_t *c, uint16_t indice, uint16_t pos, bool ultimo)
{
  uint8_t buf[PKT_FRAG_MAX];
  uint8_t n = armar_fragmento(c, indice, ultimo, buf);

  if (n == 0U) return true;
  if (!radio_canal(salto_canal(c->contacto, pos))) return false;

  esperar_hasta(c, inicio_ranura(c, pos));
  if (!radio_tx_iniciar(buf, n)) return false;
  return esperar_evento(c, CONT_T_TX_TOPE_MS, NULL) != RADIO_EV_FALLA;
}

/* Escucha el ACK en la ranura 'pos' */
static bool recibir_ack(const ctx_t *c, uint16_t pos, pkt_ack_t *ack, cont_res_t *r)
{
  radio_rx_t pkt;

  if (!radio_canal(salto_canal(c->contacto, pos))) return false;
  esperar_hasta(c, inicio_ranura(c, pos));
  if (!radio_rx_iniciar(CONT_T_ACK_RX_MS)) return false;
  if (esperar_evento(c, CONT_T_ACK_TOPE_MS, &pkt) != RADIO_EV_RX_OK) return false;
  if (!pkt_ack_leer(pkt.datos, pkt.largo, c->nodo, c->contacto, c->n_frag, ack)) return false;

  r->rssi_ack   = pkt.rssi_dbm;
  r->snr_ack_x4 = pkt.snr_x4;
  return true;
}

/* ¿Llego entero el periodo i segun el ACK? */
static bool periodo_completo(const ctx_t *c, uint8_t i, const pkt_ack_t *ack)
{
  if (ack->completo) return true;
  for (uint16_t f = c->per[i].frag0; f < c->per[i].frag0 + c->per[i].nfrag; f++)
  {
    if (pkt_ack_falta(ack, f)) return false;
  }
  return true;
}

/* ---------- Contacto ---------- */
void contacto_ejecutar(uint8_t nodo, uint8_t contacto, bool pide_ranura,
                       void (*servicio)(void), cont_res_t *res)
{
  ctx_t      *c = &s_ctx;
  cont_res_t  vacio;
  cont_res_t *r = (res != NULL) ? res : &vacio;
  radio_rx_t  pkt;
  uint8_t     hola[PKT_HOLA_LARGO];
  bool        recibido[CONT_MAX_PERIODOS] = { false };
  uint32_t    t_ini = HAL_GetTick();

  memset(r, 0, sizeof(*r));
  memset(c, 0, sizeof(*c));
  c->nodo     = nodo;
  c->contacto = contacto;
  c->servicio = servicio;

  armar_plan(c);
  r->periodos   = c->n_per;
  r->fragmentos = c->n_frag;

  /* ---- 1. hola / te oigo en el canal de encuentro ---- */
  r->estado = CONT_FALLA_RADIO;
  if (!radio_despertar())                          goto fin;
  if (!radio_canal(SALTO_CANAL_ENCUENTRO))         goto fin;
  (void)pkt_hola_armar(hola, nodo, contacto, c->n_frag, pide_ranura);
  if (!radio_tx_iniciar(hola, PKT_HOLA_LARGO))     goto fin;
  if (esperar_evento(c, CONT_T_TX_TOPE_MS, NULL) != RADIO_EV_TX_OK) goto fin;

  r->estado = CONT_SIN_TEOIGO;
  if (!radio_rx_iniciar(CONT_T_TEOIGO_MS))         goto fin;
  if (esperar_evento(c, CONT_T_TEOIGO_MS + 300U, &pkt) != RADIO_EV_RX_OK ||
      !pkt_teoigo_valido(pkt.datos, pkt.largo, nodo, contacto))
  {
    goto fin;                                      /* sin te oigo no se manda la rafaga */
  }
  r->rssi_teoigo   = pkt.rssi_dbm;
  r->snr_teoigo_x4 = pkt.snr_x4;
  c->t0 = HAL_GetTick() + CONT_T_GUARDA_MS;

  /* ---- 2. rafaga ---- */
  uint16_t pos = 0U;
  r->estado = CONT_FALLA_RADIO;
  for (uint16_t i = 0; i < c->n_frag; i++, pos++)
  {
    if (!enviar(c, i, pos, i + 1U == c->n_frag)) goto fin;
  }

  /* ---- 3. ACK y reenvios ---- */
  r->estado = CONT_SIN_ACK;
  for (;;)
  {
    pkt_ack_t ack;
    bool ok = recibir_ack(c, pos++, &ack, r);
    if (!ok) break;

    r->estado    = CONT_OK;
    r->completo  = ack.completo;
    r->hora_gw   = ack.hora;
    r->comando   = ack.comando;
    r->argumento = ack.argumento;

    for (uint8_t i = 0; i < c->n_per; i++)
    {
      if (!recibido[i] && periodo_completo(c, i, &ack)) recibido[i] = true;
    }

    if (ack.completo || r->rondas >= CONT_RONDAS_REENVIO || r->reenviados >= CONT_MAX_REENVIOS) break;

    /* Ronda de reenvio: los faltantes en orden, sin pasar el tope de 32 */
    uint16_t faltan[CONT_MAX_REENVIOS];
    uint8_t  nf = 0U;
    for (uint16_t f = 0; f < c->n_frag && (r->reenviados + nf) < CONT_MAX_REENVIOS; f++)
    {
      if (pkt_ack_falta(&ack, f)) faltan[nf++] = f;
    }
    if (nf == 0U) break;

    for (uint8_t j = 0; j < nf; j++, pos++)
    {
      if (!enviar(c, faltan[j], pos, j + 1U == nf)) { r->estado = CONT_FALLA_RADIO; goto borrar; }
    }
    r->reenviados = (uint16_t)(r->reenviados + nf);
    r->rondas++;
  }

borrar:
  /* ---- 4. se borra solo lo que llego completo (y solo si hubo ACK) ---- */
  for (uint8_t i = 0; i < c->n_per; i++)
  {
    if (recibido[i] && almacen_borrar(c->per[i].n_periodo)) r->borrados++;
  }

fin:
  radio_abortar();
  (void)radio_dormir();
  r->duracion_ms = HAL_GetTick() - t_ini;
}
#endif /* BOLO_HOST */
