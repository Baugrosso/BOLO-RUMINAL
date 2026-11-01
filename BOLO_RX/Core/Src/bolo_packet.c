/**
 * @file    bolo_packet.c
 * @brief   Arma y desarma el paquete del bolo, byte por byte (ver el mapa en
 *          bolo_packet.h). No se manda un struct directo porque su relleno y su
 *          orden de bytes dependen del compilador: asi el formato lo fija la tabla.
 */
#include "bolo_packet.h"
#include <stddef.h>

/* ---------- Escribir / leer numeros de 16 bits (little-endian) ---------- */
static void poner_u16(uint8_t *b, uint16_t v)
{
  b[0] = (uint8_t)(v & 0xFFU);   /* byte bajo primero */
  b[1] = (uint8_t)(v >> 8);
}

static uint16_t sacar_u16(const uint8_t *b)
{
  return (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

/* ===================== Armar ===================== */
uint8_t bolo_pkt_encode(const bolo_pkt_t *p, uint8_t *buf)
{
  if (p == NULL || buf == NULL) return 0U;

  buf[0] = (uint8_t)((PKT_VERSION << 4) | (PKT_TIPO_DATOS & 0x0FU));
  buf[1] = p->nodo;
  poner_u16(&buf[2], p->seq);

  buf[4] = (uint8_t)((p->mov & 0x03U) | ((p->term & 0x07U) << 2));
  buf[5] = (uint8_t)((p->temp_ok         ? 0x01U : 0U) |
                     (p->ventana_cortada ? 0x02U : 0U) |
                     (p->bus_recuperado  ? 0x04U : 0U) |
                     (p->evento_fin      ? 0x08U : 0U) |
                     ((p->reinicio & 0x07U) << 4)      |
                     (p->evento_suba     ? 0x80U : 0U));

  poner_u16(&buf[6],  p->desvio_ejes_dmg);
  poner_u16(&buf[8],  p->desvio_mag_dmg);
  poner_u16(&buf[10], (uint16_t)p->temp_centi);     /* el signo viaja en complemento a 2 */
  poner_u16(&buf[12], (uint16_t)p->delta_centi);
  buf[14] = p->hum_pct;
  buf[15] = p->evento_fin ? p->ev_ciclos : 0U;
  poner_u16(&buf[16], (uint16_t)(p->evento_fin ? p->ev_extremo_centi : 0));

  return PKT_LARGO_V1;
}

/* ===================== Desarmar ===================== */
bool bolo_pkt_decode(const uint8_t *buf, uint8_t largo, bolo_pkt_t *p)
{
  if (buf == NULL || p == NULL || largo != PKT_LARGO_V1) return false;

  uint8_t version = (uint8_t)(buf[0] >> 4);
  uint8_t tipo    = (uint8_t)(buf[0] & 0x0FU);
  if (version != PKT_VERSION || tipo != PKT_TIPO_DATOS) return false;

  p->version = version;
  p->tipo    = tipo;
  p->nodo    = buf[1];
  p->seq     = sacar_u16(&buf[2]);

  p->mov             = (uint8_t)(buf[4] & 0x03U);
  p->term            = (uint8_t)((buf[4] >> 2) & 0x07U);
  p->temp_ok         = (buf[5] & 0x01U) != 0U;
  p->ventana_cortada = (buf[5] & 0x02U) != 0U;
  p->bus_recuperado  = (buf[5] & 0x04U) != 0U;
  p->evento_fin      = (buf[5] & 0x08U) != 0U;
  p->reinicio        = (uint8_t)((buf[5] >> 4) & 0x07U);
  p->evento_suba     = (buf[5] & 0x80U) != 0U;

  p->desvio_ejes_dmg  = sacar_u16(&buf[6]);
  p->desvio_mag_dmg   = sacar_u16(&buf[8]);
  p->temp_centi       = (int16_t)sacar_u16(&buf[10]);
  p->delta_centi      = (int16_t)sacar_u16(&buf[12]);
  p->hum_pct          = buf[14];
  p->ev_ciclos        = buf[15];
  p->ev_extremo_centi = (int16_t)sacar_u16(&buf[16]);
  return true;
}

/* ===================== Textos ===================== */
const char *bolo_pkt_mov_str(uint8_t mov)
{
  switch (mov)
  {
    case PKT_MOV_REPOSO: return "REPOSO";
    case PKT_MOV_SUAVE:  return "MOV. SUAVE";
    case PKT_MOV_FUERTE: return "MOV. FUERTE";
    default:             return "DESCONOCIDO";
  }
}

const char *bolo_pkt_term_str(uint8_t term)
{
  switch (term)
  {
    case PKT_TERM_APRENDE: return "aprendiendo";
    case PKT_TERM_NORMAL:  return "normal";
    case PKT_TERM_BAJADA:  return "BAJADA (posible toma de agua)";
    case PKT_TERM_SUBA:    return "SUBA";
    default:               return "sin dato";
  }
}

const char *bolo_pkt_rst_str(uint8_t rst)
{
  switch (rst)
  {
    case PKT_RST_ENCENDIDO: return "encendido";
    case PKT_RST_BOTON:     return "boton de reset";
    case PKT_RST_SOFTWARE:  return "software / carga desde CubeIDE";
    case PKT_RST_WATCHDOG:  return "WATCHDOG";
    case PKT_RST_HARDFAULT: return "HARDFAULT";
    default:                return "desconocida";
  }
}
