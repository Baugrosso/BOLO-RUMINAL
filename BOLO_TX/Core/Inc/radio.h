/**
 * @file    radio.h
 * @brief   Interfaz generica de la radio LoRa del bolo. La aplicacion usa estas
 *          funciones y no nombra al chip. En el bolo final (STM32WLE5, radio
 *          interna) cambia solo lo que esta debajo de esta interfaz.
 *
 *  Parametros (T9-T11): SF9, 125 kHz, CR 4/5, preambulo de 8, encabezado
 *  explicito, CRC; payload maximo 66 B (~390 ms). +22 dBm fijo. TCXO por DIO3 a
 *  1,8 V y DIO2 como control del conmutador de RF.
 *
 *  Las operaciones no bloquean: se arranca una TX o RX y despues se consulta
 *  radio_evento() mientras el micro atiende el muestreo.
 */
#ifndef RADIO_H
#define RADIO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define RADIO_MAX_PAYLOAD   66U      /* bytes maximos por paquete (SF9) */

/* Pasos de radio_init(), en orden. Si uno falla, queda anotado cual. */
typedef enum
{
  RADIO_PASO_NINGUNO = 0,
  RADIO_PASO_RESET,           /*  1. reset por hardware                     */
  RADIO_PASO_LEER_ERR,        /*  2. leer errores del arranque              */
  RADIO_PASO_BORRAR_ERR,      /*  3. borrar esos errores                    */
  RADIO_PASO_STBY_RC,         /*  4. standby con el reloj RC interno        */
  RADIO_PASO_REGULADOR,       /*  5. regulador DC-DC                        */
  RADIO_PASO_TCXO,            /*  6. DIO3 alimenta al TCXO                  */
  RADIO_PASO_CALIB,           /*  7. calibracion completa                   */
  RADIO_PASO_CALIB_IMG,       /*  8. calibracion de la banda 902-928        */
  RADIO_PASO_ANTENA,          /*  9. DIO2 maneja la llave de antena         */
  RADIO_PASO_LORA,            /* 10. modo LoRa                              */
  RADIO_PASO_FRECUENCIA,      /* 11. frecuencia (canal de encuentro)        */
  RADIO_PASO_POTENCIA,        /* 12. amplificador, potencia y OCP           */
  RADIO_PASO_MODULACION,      /* 13. memoria, SF/BW/CR y formato de paquete */
  RADIO_PASO_IRQ,             /* 14. que eventos avisa DIO1                 */
  RADIO_PASO_STBY_XOSC,       /* 15. standby con el TCXO encendido          */
  RADIO_PASO_VERIFICAR        /* 16. errores = 0 y modo = STBY_XOSC         */
} radio_paso_t;

typedef struct
{
  bool         ok;
  radio_paso_t paso_fallido;
  uint16_t     err_arranque;
  uint16_t     err_final;
  uint8_t      estado;
} radio_diag_t;

typedef enum
{
  RADIO_EV_NINGUNO = 0,       /* todavia nada                              */
  RADIO_EV_TX_OK,             /* termino de transmitir                     */
  RADIO_EV_RX_OK,             /* llego un paquete sano (pkt completo)      */
  RADIO_EV_RX_ERROR,          /* llego algo roto (CRC o encabezado)        */
  RADIO_EV_TIMEOUT,           /* vencio el tiempo de TX o de RX            */
  RADIO_EV_FALLA              /* fallo la comunicacion con el chip         */
} radio_ev_t;

typedef struct
{
  uint8_t  largo;
  uint8_t  datos[RADIO_MAX_PAYLOAD];
  int16_t  rssi_dbm;          /* potencia con que llego [dBm]               */
  int8_t   snr_x4;            /* relacion senal/ruido x4 (38 = 9,5 dB)      */
} radio_rx_t;

bool        radio_init(SPI_HandleTypeDef *hspi, int8_t potencia_dbm, radio_diag_t *diag);
bool        radio_dormir(void);                  /* sleep conservando la configuracion   */
bool        radio_despertar(void);               /* false: hay que repetir radio_init    */
bool        radio_potencia(int8_t dbm);          /* -9..+22 dBm                          */
bool        radio_canal(uint8_t canal);          /* 0..63 (ver salto.h)                  */

bool        radio_tx_iniciar(const uint8_t *datos, uint8_t largo);
bool        radio_rx_iniciar(uint32_t timeout_ms);
radio_ev_t  radio_evento(radio_rx_t *pkt);       /* se consulta hasta que no sea NINGUNO */
void        radio_abortar(void);                 /* corta TX/RX y queda en standby       */

/* Ayudas para imprimir */
const char *radio_paso_str(radio_paso_t paso);
const char *radio_modo_str(uint8_t estado);

#endif /* RADIO_H */
