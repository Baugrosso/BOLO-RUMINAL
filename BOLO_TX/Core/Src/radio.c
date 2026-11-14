/**
 * @file    radio.c
 * @brief   Implementacion de la radio sobre el modulo Wio-SX1262 (drv_sx1262).
 */
#include "radio.h"
#include "drv_sx1262.h"
#include "salto.h"
#include <stddef.h>

/* ===================== Configuracion =====================
 * Tienen que ser IGUALES en el bolo y en la gateway: si difieren en SF, BW, CR,
 * sync word o canales, no se escuchan. */

/* ---- Hardware del modulo Wio-SX1262 (hoja de datos de Seeed) ---- */
#define RADIO_TCXO_TENSION    SX1262_TCXO_1V8   /* >= 200 mV por debajo de VCC      */
#define RADIO_TCXO_DEMORA_US  5000U             /* 5 ms para que se estabilice      */
#define RADIO_REGULADOR       SX1262_REGULADOR_DCDC

/* ---- LoRa (T9) ---- */
#define RADIO_SF              9U                /* SF9 obligatorio                  */
#define RADIO_BW              SX1262_BW_125     /* 125 kHz                          */
#define RADIO_CR              SX1262_CR_4_5     /* 4/5                              */
#define RADIO_LDRO            false             /* simbolo de 4,1 ms: no hace falta */
#define RADIO_PREAMBULO       8U
#define RADIO_CRC             true
#define RADIO_RX_LARGO_MAX    255U              /* en RX el largo real viene en el encabezado */

/* ---- Potencia (T11): PA armado para +22 dBm; SetTxParams fija los dBm ---- */
#define RADIO_PA_DUTY         0x04U
#define RADIO_PA_HPMAX        0x07U
#define RADIO_RAMPA           SX1262_RAMPA_200US

/* ---- Tiempos ---- */
#define RADIO_TX_TIMEOUT_US   600000U           /* el chip corta solo (paquete de 66 B: ~390 ms) */

#define RADIO_IRQ_USADAS      (SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE | \
                               SX1262_IRQ_HEADER_ERR | SX1262_IRQ_CRC_ERR | \
                               SX1262_IRQ_TIMEOUT)

/* ===================== Estado ===================== */
static bool s_dormida = false;
static bool s_en_tx   = false;

/* Llave de antena del modulo (pin RF_SW). La que conmuta TX/RX es DIO2, que la
 * maneja el chip solo; RF_SW se pone en bajo para transmitir y en alto para recibir. */
static void antena_tx(bool transmitir)
{
  HAL_GPIO_WritePin(LORA_RFSW_GPIO_Port, LORA_RFSW_Pin,
                    transmitir ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

#define PASO(n, llamada)                                                 \
  do {                                                                   \
    if ((llamada) != SX_OK) { d->paso_fallido = (n); return false; }     \
  } while (0)

/* ===================== Encendido ===================== */
bool radio_init(SPI_HandleTypeDef *hspi, int8_t potencia_dbm, radio_diag_t *diag)
{
  radio_diag_t  vacio = { 0 };
  radio_diag_t *d     = (diag != NULL) ? diag : &vacio;
  uint8_t       ocp   = SX1262_OCP_140MA;

  *d = (radio_diag_t){ 0 };
  s_dormida = false;
  s_en_tx   = false;

  PASO(RADIO_PASO_RESET,      sx1262_init(hspi));
  PASO(RADIO_PASO_LEER_ERR,   sx1262_get_errors(&d->err_arranque));
  PASO(RADIO_PASO_BORRAR_ERR, sx1262_clear_errors());

  PASO(RADIO_PASO_STBY_RC,    sx1262_set_standby(SX1262_STDBY_RC));
  PASO(RADIO_PASO_REGULADOR,  sx1262_set_regulator(RADIO_REGULADOR));
  PASO(RADIO_PASO_TCXO,       sx1262_set_tcxo(RADIO_TCXO_TENSION, RADIO_TCXO_DEMORA_US));

  PASO(RADIO_PASO_CALIB,      sx1262_calibrate(SX1262_CALIB_TODO));
  PASO(RADIO_PASO_CALIB_IMG,  sx1262_calibrate_image(SX1262_IMG_902_928_F1,
                                                     SX1262_IMG_902_928_F2));

  PASO(RADIO_PASO_ANTENA,     sx1262_set_dio2_rf_switch(true));

  PASO(RADIO_PASO_LORA,       sx1262_set_packet_type(SX1262_PAQUETE_LORA));
  PASO(RADIO_PASO_FRECUENCIA, sx1262_set_rf_frequency(salto_hz(SALTO_CANAL_ENCUENTRO)));
  PASO(RADIO_PASO_POTENCIA,   sx1262_set_pa_config(RADIO_PA_DUTY, RADIO_PA_HPMAX));
  PASO(RADIO_PASO_POTENCIA,   sx1262_write_register(SX1262_REG_OCP, &ocp, 1U));
  PASO(RADIO_PASO_POTENCIA,   sx1262_set_tx_params(potencia_dbm, RADIO_RAMPA));
  PASO(RADIO_PASO_MODULACION, sx1262_set_buffer_base(0x00U, 0x00U));
  PASO(RADIO_PASO_MODULACION, sx1262_set_lora_modulation(RADIO_SF, RADIO_BW,
                                                         RADIO_CR, RADIO_LDRO));
  PASO(RADIO_PASO_MODULACION, sx1262_set_lora_packet(RADIO_PREAMBULO,
                                                     RADIO_RX_LARGO_MAX, RADIO_CRC));
  /* Entre operaciones de un contacto el TCXO queda prendido: cada ranura dura
   * 0,4 s y el fragmento ~390 ms, no hay lugar para esperar 5 ms de arranque. */
  PASO(RADIO_PASO_MODULACION, sx1262_set_fallback(SX1262_FALLBACK_STDBY_XOSC));
  PASO(RADIO_PASO_IRQ,        sx1262_set_dio_irq(RADIO_IRQ_USADAS, RADIO_IRQ_USADAS));

  antena_tx(false);
  PASO(RADIO_PASO_STBY_XOSC,  sx1262_set_standby(SX1262_STDBY_XOSC));

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

/* ===================== Ajustes ===================== */
bool radio_potencia(int8_t dbm)
{
  if (dbm < -9)  dbm = -9;
  if (dbm > 22)  dbm = 22;
  return sx1262_set_tx_params(dbm, RADIO_RAMPA) == SX_OK;
}

bool radio_canal(uint8_t canal)
{
  return sx1262_set_rf_frequency(salto_hz(canal)) == SX_OK;
}

/* ===================== TX / RX ===================== */
bool radio_tx_iniciar(const uint8_t *datos, uint8_t largo)
{
  if (datos == NULL || largo == 0U || largo > RADIO_MAX_PAYLOAD) return false;

  antena_tx(true);
  if (sx1262_set_lora_packet(RADIO_PREAMBULO, largo, RADIO_CRC) != SX_OK ||
      sx1262_write_buffer(0x00U, datos, largo)                  != SX_OK ||
      sx1262_clear_irq(SX1262_IRQ_TODAS)                         != SX_OK ||
      sx1262_set_tx(RADIO_TX_TIMEOUT_US)                         != SX_OK)
  {
    antena_tx(false);
    return false;
  }
  s_en_tx = true;
  return true;
}

/* RX de un disparo: el chip corta solo si en 'timeout_ms' no detecto un paquete;
 * si detecto el encabezado, espera a que termine. */
bool radio_rx_iniciar(uint32_t timeout_ms)
{
  uint32_t pasos = (timeout_ms * 64U);              /* pasos de 15,625 us */
  if (pasos == 0U || pasos >= SX1262_RX_CONTINUO) pasos = SX1262_RX_CONTINUO - 1U;

  antena_tx(false);
  s_en_tx = false;
  return (sx1262_set_lora_packet(RADIO_PREAMBULO, RADIO_RX_LARGO_MAX, RADIO_CRC) == SX_OK) &&
         (sx1262_clear_irq(SX1262_IRQ_TODAS)                                    == SX_OK) &&
         (sx1262_set_rx(pasos)                                                   == SX_OK);
}

radio_ev_t radio_evento(radio_rx_t *pkt)
{
  uint16_t irq = 0U;

  if (!sx1262_dio1()) return RADIO_EV_NINGUNO;

  if (sx1262_get_irq(&irq) != SX_OK) return RADIO_EV_FALLA;
  (void)sx1262_clear_irq(SX1262_IRQ_TODAS);

  if (s_en_tx)
  {
    s_en_tx = false;
    antena_tx(false);
    if ((irq & SX1262_IRQ_TX_DONE) != 0U) return RADIO_EV_TX_OK;
    return RADIO_EV_TIMEOUT;
  }

  if ((irq & SX1262_IRQ_TIMEOUT) != 0U) return RADIO_EV_TIMEOUT;
  if ((irq & (SX1262_IRQ_CRC_ERR | SX1262_IRQ_HEADER_ERR)) != 0U ||
      (irq & SX1262_IRQ_RX_DONE) == 0U)
  {
    return RADIO_EV_RX_ERROR;
  }

  if (pkt != NULL)
  {
    uint8_t largo = 0U, inicio = 0U;
    *pkt = (radio_rx_t){ 0 };
    (void)sx1262_get_packet_status(&pkt->rssi_dbm, &pkt->snr_x4);
    if (sx1262_get_rx_buffer_status(&largo, &inicio) != SX_OK ||
        largo == 0U || largo > RADIO_MAX_PAYLOAD ||
        sx1262_read_buffer(inicio, pkt->datos, largo) != SX_OK)
    {
      return RADIO_EV_RX_ERROR;
    }
    pkt->largo = largo;
  }
  return RADIO_EV_RX_OK;
}

void radio_abortar(void)
{
  s_en_tx = false;
  antena_tx(false);
  (void)sx1262_set_standby(SX1262_STDBY_XOSC);
  (void)sx1262_clear_irq(SX1262_IRQ_TODAS);
}

/* ===================== Bajo consumo ===================== */
/* Duerme la radio conservando la configuracion (warm start). RF_SW queda en bajo. */
bool radio_dormir(void)
{
  if (s_dormida) return true;
  if (sx1262_set_sleep(true) != SX_OK) return false;
  HAL_GPIO_WritePin(LORA_RFSW_GPIO_Port, LORA_RFSW_Pin, GPIO_PIN_RESET);
  s_dormida = true;
  return true;
}

/* La despierta y verifica que quede en standby RC. Si devuelve false, la
 * configuracion no es confiable: repetir radio_init(). */
bool radio_despertar(void)
{
  uint8_t estado = 0;

  if (!s_dormida) return true;
  s_dormida = false;

  if (sx1262_wakeup() != SX_OK)            return false;
  if (sx1262_get_status(&estado) != SX_OK) return false;
  antena_tx(false);
  return (SX1262_MODO(estado) == SX1262_MODO_STBY_RC);
}

/* ===================== Textos ===================== */
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
