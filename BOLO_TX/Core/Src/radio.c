/**
 * @file    radio.c
 * @brief   Implementacion de la radio sobre el modulo Wio-SX1262 (drv_sx1262).
 *          Etapa 2a: secuencia de encendido.
 *          Etapa 2b: configuracion LoRa y transmision.
 *          Etapa 2c: recepcion continua.
 */
#include "radio.h"
#include "drv_sx1262.h"
#include <stddef.h>

/* ===================== Configuracion =====================
 * Los parametros LoRa tienen que ser IGUALES en el TX y en el RX: si difieren en
 * frecuencia, SF, BW, CR o sync word, no se escuchan. Por eso viven aca, en un
 * archivo que es identico en los dos proyectos. */

/* ---- Hardware del modulo Wio-SX1262 (hoja de datos de Seeed) ---- */
#define RADIO_TCXO_TENSION    SX1262_TCXO_1V8   /* 1,7-3,3 V y 200 mV menos que VCC */
#define RADIO_TCXO_DEMORA_US  5000U             /* 5 ms para que se estabilice      */
#define RADIO_REGULADOR       SX1262_REGULADOR_DCDC

/* ---- LoRa ---- */
#define RADIO_FREC_HZ         915000000UL       /* banda 902-928 MHz (Argentina)    */
#define RADIO_SF              7U                /* factor de dispersion             */
#define RADIO_BW              SX1262_BW_125     /* ancho de banda 125 kHz           */
#define RADIO_CR              SX1262_CR_4_5     /* tasa de codigo 4/5               */
#define RADIO_LDRO            false             /* solo hace falta con SF11-12/125k */
#define RADIO_PREAMBULO       8U                /* simbolos de preambulo            */
#define RADIO_CRC             true              /* el receptor descarta paquetes rotos */

/* ---- Potencia ----
 * Banco: 0 dBm (la entrada del receptor aguanta hasta 0 dBm). Con el PA armado
 * para +22 dBm (0x04, 0x07), SetTxParams fija la potencia directamente en dBm.
 * Paso de bateria: a potencia baja conviene la tabla de +14 dBm, que es mas eficiente. */
#define RADIO_PA_DUTY         0x04U
#define RADIO_PA_HPMAX        0x07U
#define RADIO_POTENCIA_DBM    0
#define RADIO_RAMPA           SX1262_RAMPA_200US

/* ---- Tiempos de transmision ----
 * Un paquete de ~10 bytes con SF7/125 kHz tarda ~40 ms en el aire. */
#define RADIO_TX_TIMEOUT_US   1000000U          /* el chip corta solo si pasa 1 s   */
#define RADIO_TX_ESPERA_MS    1200U             /* y nosotros esperamos un poco mas */

#define RADIO_IRQ_USADAS      (SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE | \
                               SX1262_IRQ_HEADER_ERR | SX1262_IRQ_CRC_ERR | \
                               SX1262_IRQ_TIMEOUT)

/* ===================== Funciones privadas ===================== */
/* Llave de antena del modulo (pin RF_SW). La que conmuta TX/RX es DIO2, que la
 * maneja el chip solo; RF_SW se pone en bajo para transmitir y en alto para recibir. */
static void antena_tx(bool transmitir)
{
  HAL_GPIO_WritePin(LORA_RFSW_GPIO_Port, LORA_RFSW_Pin,
                    transmitir ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* true mientras el chip esta en sleep: cualquier comando primero lo despierta */
static bool s_dormida = false;

/* Si la llamada fallo, anota el paso y corta radio_init() */
#define PASO(n, llamada)                                                 \
  do {                                                                   \
    if ((llamada) != SX_OK) { d->paso_fallido = (n); return false; }     \
  } while (0)

/* ===================== Funciones publicas ===================== */
bool radio_init(SPI_HandleTypeDef *hspi, radio_diag_t *diag)
{
  radio_diag_t  vacio = { 0 };
  radio_diag_t *d     = (diag != NULL) ? diag : &vacio;

  *d = (radio_diag_t){ 0 };
  s_dormida = false;            /* el reset por hardware lo despierta igual */

  /* 1-3. Reset y "foto" de los errores con que arranca el chip */
  PASO(RADIO_PASO_RESET,      sx1262_init(hspi));
  PASO(RADIO_PASO_LEER_ERR,   sx1262_get_errors(&d->err_arranque));
  PASO(RADIO_PASO_BORRAR_ERR, sx1262_clear_errors());

  /* 4-6. En standby RC (reloj interno): regulador y alimentacion del TCXO */
  PASO(RADIO_PASO_STBY_RC,    sx1262_set_standby(SX1262_STDBY_RC));
  PASO(RADIO_PASO_REGULADOR,  sx1262_set_regulator(RADIO_REGULADOR));
  PASO(RADIO_PASO_TCXO,       sx1262_set_tcxo(RADIO_TCXO_TENSION, RADIO_TCXO_DEMORA_US));

  /* 7-8. Con el TCXO configurado se recalibra todo (el datasheet lo pide) y la banda */
  PASO(RADIO_PASO_CALIB,      sx1262_calibrate(SX1262_CALIB_TODO));
  PASO(RADIO_PASO_CALIB_IMG,  sx1262_calibrate_image(SX1262_IMG_902_928_F1,
                                                     SX1262_IMG_902_928_F2));

  /* 9. La llave de antena la maneja el chip con DIO2 */
  PASO(RADIO_PASO_ANTENA,     sx1262_set_dio2_rf_switch(true));

  /* 10-14. Configuracion LoRa, en el orden que da el datasheet */
  PASO(RADIO_PASO_LORA,       sx1262_set_packet_type(SX1262_PAQUETE_LORA));
  PASO(RADIO_PASO_FRECUENCIA, sx1262_set_rf_frequency(RADIO_FREC_HZ));
  PASO(RADIO_PASO_POTENCIA,   sx1262_set_pa_config(RADIO_PA_DUTY, RADIO_PA_HPMAX));
  PASO(RADIO_PASO_POTENCIA,   sx1262_set_tx_params(RADIO_POTENCIA_DBM, RADIO_RAMPA));
  PASO(RADIO_PASO_MODULACION, sx1262_set_buffer_base(0x00U, 0x00U));
  PASO(RADIO_PASO_MODULACION, sx1262_set_lora_modulation(RADIO_SF, RADIO_BW,
                                                         RADIO_CR, RADIO_LDRO));
  PASO(RADIO_PASO_MODULACION, sx1262_set_lora_packet(RADIO_PREAMBULO,
                                                     RADIO_MAX_PAYLOAD, RADIO_CRC));
  PASO(RADIO_PASO_IRQ,        sx1262_set_dio_irq(RADIO_IRQ_USADAS, RADIO_IRQ_USADAS));

  /* 15. Standby con el TCXO encendido: punto de partida para TX y RX */
  antena_tx(false);
  PASO(RADIO_PASO_STBY_XOSC,  sx1262_set_standby(SX1262_STDBY_XOSC));

  /* 16. Verificacion: sin errores internos y en el modo esperado */
  PASO(RADIO_PASO_VERIFICAR,  sx1262_get_errors(&d->err_final));
  PASO(RADIO_PASO_VERIFICAR,  sx1262_get_status(&d->estado));

  if (d->err_final != 0U || SX1262_MODO(d->estado) != SX1262_MODO_STBY_XOSC)
  {
    d->paso_fallido = RADIO_PASO_VERIFICAR;
    return false;
  }

  d->ok = true;
  return true;
}

/* Manda un paquete y espera a que el chip avise que termino (DIO1).
 * Secuencia del datasheet: largo del paquete -> cargar la memoria -> borrar
 * avisos viejos -> SetTx -> esperar DIO1 -> leer y borrar el aviso. */
bool radio_send(const uint8_t *datos, uint8_t largo, radio_tx_t *info)
{
  radio_tx_t  vacio = { 0 };
  radio_tx_t *t     = (info != NULL) ? info : &vacio;

  *t = (radio_tx_t){ 0 };

  if (datos == NULL || largo == 0U || largo > RADIO_MAX_PAYLOAD)
  {
    t->res = RADIO_TX_ERR_PARAM;
    return false;
  }

  antena_tx(true);

  if (sx1262_set_lora_packet(RADIO_PREAMBULO, largo, RADIO_CRC) != SX_OK ||
      sx1262_write_buffer(0x00U, datos, largo)                  != SX_OK ||
      sx1262_clear_irq(SX1262_IRQ_TODAS)                         != SX_OK ||
      sx1262_set_tx(RADIO_TX_TIMEOUT_US)                         != SX_OK)
  {
    antena_tx(false);
    t->res = RADIO_TX_ERR_SPI;
    return false;
  }

  /* Esperar el aviso del chip (DIO1 sube con TxDone o con Timeout) */
  uint32_t t0 = HAL_GetTick();
  while (!sx1262_dio1())
  {
    if ((HAL_GetTick() - t0) > RADIO_TX_ESPERA_MS) break;
  }
  t->ms = HAL_GetTick() - t0;

  (void)sx1262_get_irq(&t->irq);
  (void)sx1262_clear_irq(SX1262_IRQ_TODAS);
  antena_tx(false);

  t->res = ((t->irq & SX1262_IRQ_TX_DONE) != 0U) ? RADIO_TX_OK : RADIO_TX_ERR_TIMEOUT;
  return (t->res == RADIO_TX_OK);
}

/* Deja el chip escuchando en forma continua. Despues de cada paquete sigue en RX
 * solo, asi que se llama una vez. En recepcion, el "largo" de SetPacketParams es
 * el maximo aceptado: el largo real viene en el encabezado de cada paquete. */
bool radio_rx_start(void)
{
  antena_tx(false);

  return (sx1262_set_lora_packet(RADIO_PREAMBULO, RADIO_MAX_PAYLOAD, RADIO_CRC) == SX_OK) &&
         (sx1262_clear_irq(SX1262_IRQ_TODAS)                                   == SX_OK) &&
         (sx1262_set_rx(SX1262_RX_CONTINUO)                                     == SX_OK);
}

/* Revisa si llego algo. DIO1 sube con RxDone (y tambien con errores de CRC o de
 * encabezado). Si llego: se leen el largo, los bytes y la calidad de la señal. */
bool radio_rx_poll(radio_rx_t *pkt)
{
  uint8_t largo  = 0;
  uint8_t inicio = 0;

  if (pkt == NULL || !sx1262_dio1()) return false;     /* nada nuevo */

  *pkt = (radio_rx_t){ 0 };
  (void)sx1262_get_irq(&pkt->irq);
  (void)sx1262_clear_irq(SX1262_IRQ_TODAS);

  /* RSSI y SNR del ultimo paquete (sirven tambien si llego roto) */
  (void)sx1262_get_packet_status(&pkt->rssi_dbm, &pkt->snr_x4);

  pkt->crc_ok = ((pkt->irq & SX1262_IRQ_RX_DONE) != 0U) &&
                ((pkt->irq & (SX1262_IRQ_CRC_ERR | SX1262_IRQ_HEADER_ERR)) == 0U);
  if (!pkt->crc_ok) return true;                       /* llego algo, pero roto */

  if (sx1262_get_rx_buffer_status(&largo, &inicio) != SX_OK ||
      largo == 0U || largo > RADIO_MAX_PAYLOAD ||
      sx1262_read_buffer(inicio, pkt->datos, largo) != SX_OK)
  {
    pkt->crc_ok = false;
    return true;
  }

  pkt->largo = largo;
  return true;
}

/* ---------- Bajo consumo (solo lo usa el bolo) ---------- */
/* Duerme la radio conservando la configuracion (warm start). La llave de antena
 * (RF_SW) queda en bajo: con el chip dormido no conduce y el pin no alimenta nada. */
bool radio_dormir(void)
{
  if (s_dormida) return true;
  if (sx1262_set_sleep(true) != SX_OK) return false;
  HAL_GPIO_WritePin(LORA_RFSW_GPIO_Port, LORA_RFSW_Pin, GPIO_PIN_RESET);
  s_dormida = true;
  return true;
}

/* La despierta y verifica que haya quedado en standby RC, lista para transmitir.
 * Si devuelve false, la configuracion no es confiable: repetir radio_init(). */
bool radio_despertar(void)
{
  uint8_t estado = 0;

  if (!s_dormida) return true;
  s_dormida = false;

  if (sx1262_wakeup() != SX_OK)          return false;
  if (sx1262_get_status(&estado) != SX_OK) return false;
  antena_tx(false);
  return (SX1262_MODO(estado) == SX1262_MODO_STBY_RC);
}

/* ---------- Ayudas de texto ---------- */
const char *radio_paso_str(radio_paso_t paso)
{
  switch (paso)
  {
    case RADIO_PASO_NINGUNO:    return "ninguno";
    case RADIO_PASO_RESET:      return "reset";
    case RADIO_PASO_LEER_ERR:   return "leer errores";
    case RADIO_PASO_BORRAR_ERR: return "borrar errores";
    case RADIO_PASO_STBY_RC:    return "standby RC";
    case RADIO_PASO_REGULADOR:  return "regulador DC-DC";
    case RADIO_PASO_TCXO:       return "configurar TCXO";
    case RADIO_PASO_CALIB:      return "calibracion";
    case RADIO_PASO_CALIB_IMG:  return "calibracion de banda";
    case RADIO_PASO_ANTENA:     return "llave de antena (DIO2)";
    case RADIO_PASO_LORA:       return "modo LoRa";
    case RADIO_PASO_FRECUENCIA: return "frecuencia";
    case RADIO_PASO_POTENCIA:   return "potencia";
    case RADIO_PASO_MODULACION: return "modulacion y paquete";
    case RADIO_PASO_IRQ:        return "interrupciones";
    case RADIO_PASO_STBY_XOSC:  return "standby XOSC";
    case RADIO_PASO_VERIFICAR:  return "verificacion";
    default:                    return "?";
  }
}

const char *radio_error_str(uint8_t bit)
{
  switch (bit)
  {
    case 0:  return "RC64K_CALIB";
    case 1:  return "RC13M_CALIB";
    case 2:  return "PLL_CALIB";
    case 3:  return "ADC_CALIB";
    case 4:  return "IMG_CALIB";
    case 5:  return "XOSC_START";
    case 6:  return "PLL_LOCK";
    case 8:  return "PA_RAMP";
    default: return NULL;
  }
}

const char *radio_modo_str(uint8_t estado)
{
  switch (SX1262_MODO(estado))
  {
    case SX1262_MODO_STBY_RC:   return "STBY_RC";
    case SX1262_MODO_STBY_XOSC: return "STBY_XOSC";
    case SX1262_MODO_FS:        return "FS";
    case SX1262_MODO_RX:        return "RX";
    case SX1262_MODO_TX:        return "TX";
    default:                    return "?";
  }
}

const char *radio_tx_str(radio_tx_res_t res)
{
  switch (res)
  {
    case RADIO_TX_OK:          return "OK";
    case RADIO_TX_ERR_PARAM:   return "paquete invalido";
    case RADIO_TX_ERR_SPI:     return "fallo un comando";
    case RADIO_TX_ERR_TIMEOUT: return "sin aviso de fin (timeout)";
    default:                   return "?";
  }
}
