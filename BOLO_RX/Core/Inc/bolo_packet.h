/**
 * @file    bolo_packet.h
 * @brief   Formato del paquete LoRa del bolo. Es el MISMO archivo en el bolo (TX)
 *          y en el gateway (RX): el formato queda definido aca, byte por byte.
 *
 *  Version 1 (prototipo, "opcion A"): largo FIJO de 18 bytes. El resumen del
 *  evento termico va siempre (en cero si no termino ningun evento).
 *  Pendiente para el bolo final ("opcion B", nueva version): el resumen del
 *  evento se agrega solo cuando termina un evento (~15 B el resto del tiempo).
 *
 *  Mapa de bytes (los numeros de 16 bits van en little-endian: primero el byte bajo)
 *   0      version (bits 7-4) | tipo (bits 3-0)
 *   1      ID del nodo
 *   2-3    secuencia (u16)
 *   4      movimiento (bits 1-0) | estado termico (bits 4-2)
 *   5      banderas: b0 temp valida, b1 ventana cortada, b2 bus I2C recuperado,
 *                    b3 termino un evento, b4-6 causa del ultimo reinicio,
 *                    b7 el evento que termino era una SUBA (0 = BAJADA)
 *   6-7    desvio de los ejes (maximo)  u16  [0,1 mg]
 *   8-9    desvio de la magnitud        u16  [0,1 mg]
 *   10-11  temperatura                  i16  [0,01 C]
 *   12-13  delta termico (T - base)     i16  [0,01 C]
 *   14     humedad interna              u8   [1 %]
 *   15     duracion del evento          u8   [ciclos]
 *   16-17  delta extremo del evento     i16  [0,01 C]
 */
#ifndef BOLO_PACKET_H
#define BOLO_PACKET_H

#include <stdint.h>
#include <stdbool.h>

#define PKT_VERSION         1U
#define PKT_TIPO_DATOS      1U      /* datos de un ciclo de sensado */
#define PKT_LARGO_V1        18U

/* Movimiento (mismos valores que bolo_estado_t) */
#define PKT_MOV_DESCONOCIDO 0U
#define PKT_MOV_REPOSO      1U
#define PKT_MOV_SUAVE       2U
#define PKT_MOV_FUERTE      3U

/* Estado termico (mismos valores que term_estado_t) */
#define PKT_TERM_SIN_DATO   0U
#define PKT_TERM_APRENDE    1U
#define PKT_TERM_NORMAL     2U
#define PKT_TERM_BAJADA     3U
#define PKT_TERM_SUBA       4U

/* Causa del ultimo reinicio (3 bits) */
#define PKT_RST_DESCONOCIDA 0U
#define PKT_RST_ENCENDIDO   1U
#define PKT_RST_BOTON       2U
#define PKT_RST_SOFTWARE    3U      /* incluye la carga desde CubeIDE */
#define PKT_RST_WATCHDOG    4U
#define PKT_RST_HARDFAULT   5U

/* El contenido del paquete, ya en las unidades de transmision */
typedef struct
{
  uint8_t  version;
  uint8_t  tipo;
  uint8_t  nodo;
  uint16_t seq;

  uint8_t  mov;               /* PKT_MOV_*                      */
  uint8_t  term;              /* PKT_TERM_*                     */
  bool     temp_ok;
  bool     ventana_cortada;
  bool     bus_recuperado;
  bool     evento_fin;        /* termino un evento este ciclo   */
  bool     evento_suba;       /* ese evento era una SUBA        */
  uint8_t  reinicio;          /* PKT_RST_*                      */

  uint16_t desvio_ejes_dmg;   /* 0,1 mg */
  uint16_t desvio_mag_dmg;    /* 0,1 mg */
  int16_t  temp_centi;        /* 0,01 C */
  int16_t  delta_centi;       /* 0,01 C */
  uint8_t  hum_pct;           /* 1 %    */
  uint8_t  ev_ciclos;
  int16_t  ev_extremo_centi;  /* 0,01 C */
} bolo_pkt_t;

/* Arma los bytes; devuelve el largo (PKT_LARGO_V1) o 0 si buf es NULL */
uint8_t     bolo_pkt_encode(const bolo_pkt_t *p, uint8_t *buf);

/* Desarma los bytes; false si el largo, la version o el tipo no coinciden */
bool        bolo_pkt_decode(const uint8_t *buf, uint8_t largo, bolo_pkt_t *p);

/* Textos para mostrar */
const char *bolo_pkt_mov_str(uint8_t mov);
const char *bolo_pkt_term_str(uint8_t term);
const char *bolo_pkt_rst_str(uint8_t rst);

#endif /* BOLO_PACKET_H */
