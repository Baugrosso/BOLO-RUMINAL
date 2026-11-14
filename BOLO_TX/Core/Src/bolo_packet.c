/**
 * @file    bolo_packet.c
 * @brief   Arma y desarma los formatos de aire byte a byte (ver bolo_packet.h).
 *          No se manda un struct directo porque su relleno y su orden de bytes
 *          dependen del compilador: el formato lo fija este archivo.
 */
#include "bolo_packet.h"
#include <string.h>
#include <stddef.h>

static void wr16(uint8_t *b, uint16_t v) { b[0] = (uint8_t)(v >> 8); b[1] = (uint8_t)v; }
static void wr32(uint8_t *b, uint32_t v)
{
  b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16); b[2] = (uint8_t)(v >> 8); b[3] = (uint8_t)v;
}
static uint16_t rd16(const uint8_t *b) { return (uint16_t)(((uint16_t)b[0] << 8) | b[1]); }
static uint32_t rd32(const uint8_t *b)
{
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
}

/* ===================== Metadatos ===================== */
void pkt_meta_armar(uint8_t out[PKT_META_LARGO], const pkt_meta_t *m)
{
  wr16(&out[0], (uint16_t)(((uint16_t)(m->version & 0x0FU) << 12) | (m->n_epocas & 0x0FFFU)));
  wr16(&out[2], m->n_periodo);
  wr32(&out[4], m->marca);
  out[8]  = m->n_th;
  out[9]  = m->vdd_cod;
  out[10] = (uint8_t)(((m->causa_reset & 0x0FU) << 4) | (m->sin_ack & 0x0FU));
  out[11] = (uint8_t)(((m->fallas_acc & 0x0FU) << 4) | (m->fallas_temp & 0x0FU));
}

void pkt_meta_leer(const uint8_t in[PKT_META_LARGO], pkt_meta_t *m)
{
  uint16_t v0 = rd16(&in[0]);
  m->version     = (uint8_t)(v0 >> 12);
  m->n_epocas    = (uint16_t)(v0 & 0x0FFFU);
  m->n_periodo   = rd16(&in[2]);
  m->marca       = rd32(&in[4]);
  m->n_th        = in[8];
  m->vdd_cod     = in[9];
  m->causa_reset = (uint8_t)(in[10] >> 4);
  m->sin_ack     = (uint8_t)(in[10] & 0x0FU);
  m->fallas_acc  = (uint8_t)(in[11] >> 4);
  m->fallas_temp = (uint8_t)(in[11] & 0x0FU);
}

/* ===================== Saludo ===================== */
uint8_t pkt_hola_armar(uint8_t *out, uint8_t nodo, uint8_t contacto, uint16_t n_frag, bool pide_ranura)
{
  out[0] = (uint8_t)((PKT_TIPO_HOLA << 4) | PKT_PROTO_VERSION);
  out[1] = nodo;
  out[2] = contacto;
  wr16(&out[3], n_frag);
  out[5] = pide_ranura ? PKT_HOLA_PIDE_RANURA : 0U;
  return PKT_HOLA_LARGO;
}

bool pkt_teoigo_valido(const uint8_t *in, uint8_t n, uint8_t nodo, uint8_t contacto)
{
  return (in != NULL) && (n == PKT_TEOIGO_LARGO) &&
         (in[0] == (uint8_t)((PKT_TIPO_TEOIGO << 4) | PKT_PROTO_VERSION)) &&
         (in[1] == nodo) && (in[2] == contacto);
}

/* ===================== Fragmento ===================== */
void pkt_frag_cab(uint8_t out[PKT_FRAG_CAB], uint8_t nodo, uint8_t contacto,
                  uint16_t indice, bool ultimo)
{
  out[0] = nodo;
  out[1] = contacto;
  wr16(&out[2], (uint16_t)(((indice & 0x03FFU) << 6) | (ultimo ? 0x0020U : 0U)));
}

/* ===================== ACK ===================== */
bool pkt_ack_leer(const uint8_t *in, uint8_t n, uint8_t nodo, uint8_t contacto,
                  uint16_t n_frag, pkt_ack_t *a)
{
  if (in == NULL || a == NULL || n < PKT_ACK_MIN) return false;
  if (in[0] != nodo || in[1] != contacto)         return false;

  memset(a, 0, sizeof(*a));
  a->nodo      = in[0];
  a->contacto  = in[1];
  a->estado    = in[2];
  a->completo  = (in[2] & PKT_ACK_COMPLETO) != 0U;
  a->hora      = rd32(&in[3]);
  a->comando   = in[7];
  a->argumento = in[8];

  if (!a->completo)
  {
    uint16_t nb = (uint16_t)((n_frag + 7U) / 8U);
    if (nb > PKT_BITMAP_MAX || n < PKT_ACK_MIN + nb) return false;
    memcpy(a->bitmap, &in[PKT_ACK_MIN], nb);
  }
  return true;
}

bool pkt_ack_falta(const pkt_ack_t *a, uint16_t indice)
{
  if (a->completo) return false;
  if (indice >= CONT_MAX_FRAG) return false;
  return (a->bitmap[indice >> 3] & (uint8_t)(1U << (indice & 7U))) != 0U;
}
