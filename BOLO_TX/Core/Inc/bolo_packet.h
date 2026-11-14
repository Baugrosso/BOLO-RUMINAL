/**
 * @file    bolo_packet.h
 * @brief   Formatos de aire del bolo (T9-T12): metadatos del periodo, hola,
 *          te oigo, fragmento y ACK. Todo en big-endian y con los campos de bits
 *          en el orden de la lista (el primero en los bits mas altos).
 *          La descripcion completa esta en BOLO_TX/PROTOCOLO.md.
 *
 *  Metadatos del periodo (12 B):
 *    [version 4 b | cant. epocas 12 b][n. periodo 16 b][marca de tiempo 32 b]
 *    [cant. temperaturas 8 b][VDD 8 b: 1,80 V + codigo x 7 mV]
 *    [causa reset 4 b | contactos sin ACK 4 b][fallas: acelerometro 4 b | temperatura 4 b]
 *
 *  hola (bolo -> gateway, canal de encuentro, 6 B):
 *    [tipo 4 b = 1 | version 4 b][ID nodo][contacto][cant. fragmentos 16 b][banderas]
 *    banderas: bit 0 = no tengo ranura (pido hora y ranura)
 *  te oigo (gateway -> bolo, canal de encuentro, 3 B):
 *    [tipo 4 b = 2 | version 4 b][ID nodo][contacto]
 *
 *  Fragmento (<= 66 B): cabecera de 4 B + hasta 62 B de datos
 *    [ID nodo 8 b][contacto 8 b][indice 10 b | ultimo 1 b | reserva 5 b]
 *    ultimo = 1 en el ultimo fragmento de cada tanda (rafaga o ronda de reenvio)
 *
 *  ACK (gateway -> bolo, 9 B + bitmap):
 *    [ID nodo][contacto][estado][hora gateway UNIX 32 b][comando][argumento]
 *    estado bit 7 = 1: llego todo. Si es 0, sigue el bitmap de faltantes:
 *    ceil(N/8) bytes, bit i (byte i/8, bit i%8 desde el menos significativo) = 1 si
 *    falta el fragmento i. Comandos: 0 nada, 1 ranura (arg 0-179), 2 potencia (arg dBm).
 */
#ifndef BOLO_PACKET_H
#define BOLO_PACKET_H

#include <stdint.h>
#include <stdbool.h>
#include "bolo_config.h"

#define PKT_PROTO_VERSION     1U

#define PKT_META_LARGO        12U
#define PKT_HOLA_LARGO        6U
#define PKT_TEOIGO_LARGO      3U
#define PKT_FRAG_CAB          4U
#define PKT_FRAG_DATOS        62U
#define PKT_FRAG_MAX          (PKT_FRAG_CAB + PKT_FRAG_DATOS)      /* 66 */
#define PKT_ACK_MIN           9U
#define PKT_BITMAP_MAX        (CONT_MAX_FRAG / 8U)                 /* 16 */
#define PKT_ACK_MAX           (PKT_ACK_MIN + PKT_BITMAP_MAX)       /* 25 */
#define PKT_CRC_LARGO         4U

#define PKT_TIPO_HOLA         1U
#define PKT_TIPO_TEOIGO       2U
#define PKT_HOLA_PIDE_RANURA  0x01U

#define PKT_ACK_COMPLETO      0x80U
#define PKT_CMD_NADA          0U
#define PKT_CMD_RANURA        1U
#define PKT_CMD_POTENCIA      2U

_Static_assert(PKT_FRAG_MAX <= 66U, "Payload maximo con SF9: 66 B");
_Static_assert(CONT_MAX_FRAG <= 1024U && (CONT_MAX_FRAG % 8U) == 0U, "Indice de 10 bits / bitmap en bytes");

typedef struct
{
  uint8_t  version;
  uint16_t n_epocas;
  uint16_t n_periodo;
  uint32_t marca;
  uint8_t  n_th;
  uint8_t  vdd_cod;
  uint8_t  causa_reset;
  uint8_t  sin_ack;
  uint8_t  fallas_acc;
  uint8_t  fallas_temp;
} pkt_meta_t;

typedef struct
{
  uint8_t  nodo;
  uint8_t  contacto;
  uint8_t  estado;
  bool     completo;
  uint32_t hora;
  uint8_t  comando;
  uint8_t  argumento;
  uint8_t  bitmap[PKT_BITMAP_MAX];
} pkt_ack_t;

void    pkt_meta_armar(uint8_t out[PKT_META_LARGO], const pkt_meta_t *m);
void    pkt_meta_leer(const uint8_t in[PKT_META_LARGO], pkt_meta_t *m);

uint8_t pkt_hola_armar(uint8_t *out, uint8_t nodo, uint8_t contacto, uint16_t n_frag, bool pide_ranura);
bool    pkt_teoigo_valido(const uint8_t *in, uint8_t n, uint8_t nodo, uint8_t contacto);

void    pkt_frag_cab(uint8_t out[PKT_FRAG_CAB], uint8_t nodo, uint8_t contacto,
                     uint16_t indice, bool ultimo);

/* Valida y desarma un ACK para un contacto con n_frag fragmentos */
bool    pkt_ack_leer(const uint8_t *in, uint8_t n, uint8_t nodo, uint8_t contacto,
                     uint16_t n_frag, pkt_ack_t *a);
bool    pkt_ack_falta(const pkt_ack_t *a, uint16_t indice);

#endif /* BOLO_PACKET_H */
