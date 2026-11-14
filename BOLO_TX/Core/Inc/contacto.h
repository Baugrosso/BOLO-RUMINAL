/**
 * @file    contacto.h
 * @brief   Un contacto con la gateway (T9-T12). El bolo siempre inicia.
 *
 *   1. hola en el canal de encuentro; espera el te oigo (sin te oigo no hay rafaga)
 *   2. rafaga: los periodos guardados, del mas nuevo al mas viejo (hasta 4 y hasta
 *      128 fragmentos), un fragmento por ranura de 0,4 s siguiendo el salto
 *   3. ACK en la ranura siguiente. Si faltan fragmentos: hasta 2 rondas de
 *      reenvio (maximo 32 fragmentos reenviados en total), con un ACK despues de
 *      cada ronda
 *   4. se borran los periodos que llegaron completos; el resto queda guardado
 *
 *  Durante todo el contacto se llama a 'servicio' en cada espera: el muestreo
 *  sigue (el SX1262 transmite solo) y se refresca el watchdog.
 */
#ifndef CONTACTO_H
#define CONTACTO_H

#include <stdint.h>
#include <stdbool.h>

/* Tiempos del contacto (la gateway usa los mismos) */
#define CONT_T_RANURA_MS      400U    /* un fragmento por ranura (<= 0,4 s por canal) */
#define CONT_T_GUARDA_MS      50U     /* del te oigo al inicio de la ranura 0         */
#define CONT_T_TEOIGO_MS      500U    /* ventana para recibir el te oigo              */
#define CONT_T_ACK_RX_MS      400U    /* ventana de RX del ACK (desde el inicio de su ranura) */
#define CONT_T_ACK_TOPE_MS    700U    /* tope total esperando el ACK (si ya empezo a llegar)  */
#define CONT_T_TX_TOPE_MS     500U    /* tope esperando el fin de una TX              */

typedef enum
{
  CONT_OK = 0,                /* hubo ACK (completo o parcial)               */
  CONT_SIN_TEOIGO,            /* nadie contesto el hola                      */
  CONT_SIN_ACK,               /* se mando la rafaga pero no llego el ACK     */
  CONT_FALLA_RADIO            /* la radio no respondio                       */
} cont_estado_t;

typedef struct
{
  cont_estado_t estado;
  bool     completo;          /* el ultimo ACK dijo "llego todo"            */
  uint32_t hora_gw;           /* del ultimo ACK                             */
  uint8_t  comando;
  uint8_t  argumento;
  uint8_t  periodos;          /* periodos que se mandaron                   */
  uint8_t  borrados;          /* periodos que llegaron completos            */
  uint16_t fragmentos;        /* fragmentos de la rafaga                    */
  uint16_t reenviados;
  uint8_t  rondas;
  int16_t  rssi_teoigo;
  int8_t   snr_teoigo_x4;
  int16_t  rssi_ack;
  int8_t   snr_ack_x4;
  uint32_t duracion_ms;
} cont_res_t;

/* La radio tiene que estar inicializada (dormida o no). Al volver queda dormida. */
void contacto_ejecutar(uint8_t nodo, uint8_t contacto, bool pide_ranura,
                       void (*servicio)(void), cont_res_t *res);

/* Bytes de un periodo tal como viajan (metadatos + listas + CRC-32), para el
 * plan y para los tests en la PC */
uint32_t contacto_largo_periodo(uint16_t largo_nucleo, uint16_t largo_acel);
uint16_t contacto_leer_periodo(uint16_t n_periodo, uint16_t largo_nucleo, uint16_t largo_acel,
                               uint32_t crc, uint32_t offset, uint8_t *dst, uint16_t n);
uint32_t contacto_crc_periodo(uint16_t n_periodo, uint16_t largo_nucleo, uint16_t largo_acel);

#endif /* CONTACTO_H */
