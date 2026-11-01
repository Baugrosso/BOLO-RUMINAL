/**
 * @file    radio.h
 * @brief   Interfaz generica de la radio LoRa. La aplicacion usa estas funciones
 *          y no nombra al chip. Es el MISMO archivo en el bolo (TX) y en el
 *          gateway (RX). En el bolo final (STM32WLE5, radio interna) cambia solo
 *          lo que esta debajo de esta interfaz.
 *          Etapa 2a: encendido correcto del chip (TCXO + calibracion).
 *          Etapa 2b: configuracion LoRa y transmision de un paquete.
 *          Etapa 2c: recepcion continua.
 *          Etapa 4:  dormir la radio entre transmisiones (solo el bolo).
 */
#ifndef RADIO_H
#define RADIO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define RADIO_MAX_PAYLOAD   64U      /* bytes maximos por paquete */

/* Pasos de radio_init(), en orden. Si uno falla, queda anotado cual. */
typedef enum
{
  RADIO_PASO_NINGUNO = 0,     /* no fallo nada                              */
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
  RADIO_PASO_FRECUENCIA,      /* 11. frecuencia (915 MHz)                   */
  RADIO_PASO_POTENCIA,        /* 12. amplificador y potencia de salida      */
  RADIO_PASO_MODULACION,      /* 13. memoria, SF/BW/CR y formato de paquete */
  RADIO_PASO_IRQ,             /* 14. que eventos avisa DIO1                 */
  RADIO_PASO_STBY_XOSC,       /* 15. standby con el TCXO encendido          */
  RADIO_PASO_VERIFICAR        /* 16. errores = 0 y modo = STBY_XOSC         */
} radio_paso_t;

/* Diagnostico de radio_init(): todo lo que hace falta para saber que paso */
typedef struct
{
  bool         ok;            /* true si la radio quedo lista              */
  radio_paso_t paso_fallido;  /* primer paso que fallo                     */
  uint16_t     err_arranque;  /* errores internos justo despues del reset  */
  uint16_t     err_final;     /* errores despues de toda la configuracion  */
  uint8_t      estado;        /* GetStatus al final (esperado STBY_XOSC)   */
} radio_diag_t;

/* Resultado de una transmision */
typedef enum
{
  RADIO_TX_OK = 0,            /* el chip aviso "TxDone"                    */
  RADIO_TX_ERR_PARAM,         /* paquete vacio o demasiado largo           */
  RADIO_TX_ERR_SPI,           /* fallo un comando al chip                  */
  RADIO_TX_ERR_TIMEOUT        /* el chip no aviso a tiempo                 */
} radio_tx_res_t;

typedef struct
{
  radio_tx_res_t res;
  uint16_t       irq;         /* interrupciones que levanto el chip        */
  uint32_t       ms;          /* tiempo en el aire medido (SetTx -> DIO1)  */
} radio_tx_t;

/* Un paquete recibido */
typedef struct
{
  bool     crc_ok;            /* false: llego roto (CRC o encabezado), se descarta */
  uint8_t  largo;             /* bytes recibidos                                   */
  uint8_t  datos[RADIO_MAX_PAYLOAD];
  int16_t  rssi_dbm;          /* potencia con que llego [dBm]                      */
  int8_t   snr_x4;            /* relacion señal/ruido x4 (38 = 9,5 dB)             */
  uint16_t irq;               /* interrupciones que levanto el chip                */
} radio_rx_t;

bool        radio_init(SPI_HandleTypeDef *hspi, radio_diag_t *diag);
bool        radio_send(const uint8_t *datos, uint8_t largo, radio_tx_t *info);
bool        radio_rx_start(void);                     /* empieza a escuchar sin parar   */
bool        radio_rx_poll(radio_rx_t *pkt);           /* true si llego algo (bien o mal) */
bool        radio_dormir(void);                       /* sleep conservando la configuracion */
bool        radio_despertar(void);                    /* false: hay que repetir radio_init  */

/* Ayudas para imprimir (texto de cada paso, de cada error y de cada resultado) */
const char *radio_paso_str(radio_paso_t paso);
const char *radio_error_str(uint8_t bit);     /* bit 0..15; NULL si no se usa */
const char *radio_modo_str(uint8_t estado);
const char *radio_tx_str(radio_tx_res_t res);

#endif /* RADIO_H */
