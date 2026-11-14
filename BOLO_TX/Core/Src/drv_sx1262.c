/**
 * @file    drv_sx1262.c
 * @brief   Driver minimo del SX1262. Todo comando sigue la misma receta:
 *            1. esperar a que BUSY este en bajo (el chip puede escuchar)
 *            2. bajar NSS (empieza el comando)
 *            3. intercambiar los bytes por SPI (opcode + parametros / respuesta)
 *            4. subir NSS (el chip ejecuta el comando)
 */
#include "drv_sx1262.h"
#include <stddef.h>

/* ---- Tiempos ---- */
#define SX_SPI_TIMEOUT_MS       10U   /* tope de una transferencia SPI                  */
#define SX_BUSY_TIMEOUT_MS      10U   /* tope de espera de BUSY antes de un comando     */
#define SX_RESET_PULSO_MS       1U    /* RST en bajo (el datasheet pide >= 100 us)      */
#define SX_RESET_ARRANQUE_MS    20U   /* tope para que termine de arrancar tras reset   */
#define SX_CALIB_TIMEOUT_MS     50U   /* calibraciones y arranque del TCXO (~4-10 ms)   */
#define SX_DESPERTAR_MS         10U   /* tope para salir de sleep (warm start ~0,3 ms) */

/* ---- Opcodes (datasheet SX1261/2, cap. 13) ---- */
#define OP_CLEAR_IRQ_STATUS     0x02U
#define OP_CLEAR_DEVICE_ERRORS  0x07U
#define OP_SET_DIO_IRQ_PARAMS   0x08U
#define OP_WRITE_BUFFER         0x0EU
#define OP_GET_IRQ_STATUS       0x12U
#define OP_GET_RX_BUFFER_STATUS 0x13U
#define OP_GET_PACKET_STATUS    0x14U
#define OP_READ_BUFFER          0x1EU
#define OP_GET_DEVICE_ERRORS    0x17U
#define OP_READ_REGISTER        0x1DU
#define OP_WRITE_REGISTER       0x0DU
#define OP_SET_FALLBACK         0x93U
#define OP_SET_STANDBY          0x80U
#define OP_SET_RX               0x82U
#define OP_SET_SLEEP            0x84U
#define OP_SET_TX               0x83U
#define OP_SET_RF_FREQUENCY     0x86U
#define OP_SET_PACKET_TYPE      0x8AU
#define OP_SET_MODULATION       0x8BU
#define OP_SET_PACKET_PARAMS    0x8CU
#define OP_SET_TX_PARAMS        0x8EU
#define OP_SET_BUFFER_BASE      0x8FU
#define OP_SET_PA_CONFIG        0x95U
#define OP_SET_REGULATOR        0x96U
#define OP_CALIBRATE            0x89U
#define OP_SET_DIO3_AS_TCXO     0x97U
#define OP_CALIBRATE_IMAGE      0x98U
#define OP_SET_DIO2_RF_SWITCH   0x9DU
#define OP_GET_STATUS           0xC0U
#define SX_NOP                  0x00U

#define SX_MAX_LECTURA          16U   /* bytes maximos por lectura de registros  */
#define SX_MAX_PARAM            8U    /* parametros maximos de un comando simple */

static SPI_HandleTypeDef *s_hspi = NULL;

/* ===================== Funciones privadas ===================== */
/* NSS es activo en bajo: "seleccionado" = pin en 0 */
static void nss(bool seleccionado)
{
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin,
                    seleccionado ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* Espera a que el chip libere BUSY, con tope de tiempo */
static sx_status_t esperar_libre(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();

  while (HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - t0) > timeout_ms) return SX_ERR_BUSY;
  }
  return SX_OK;
}

/* Un comando completo: BUSY -> NSS abajo -> n bytes de ida y vuelta -> NSS arriba.
 * En SPI cada byte que sale por MOSI trae uno de vuelta por MISO al mismo tiempo. */
static sx_status_t transferir(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
  if (s_hspi == NULL) return SX_ERR_PARAM;
  if (esperar_libre(SX_BUSY_TIMEOUT_MS) != SX_OK) return SX_ERR_BUSY;

  nss(true);
  HAL_StatusTypeDef r = HAL_SPI_TransmitReceive(s_hspi, tx, rx, n, SX_SPI_TIMEOUT_MS);
  nss(false);

  return (r == HAL_OK) ? SX_OK : SX_ERR_SPI;
}

/* Comando "de escritura": [opcode, param0, param1, ...]; la respuesta no importa */
static sx_status_t comando(uint8_t op, const uint8_t *param, uint8_t n)
{
  uint8_t tx[1U + SX_MAX_PARAM] = { 0 };
  uint8_t rx[1U + SX_MAX_PARAM] = { 0 };

  if (n > SX_MAX_PARAM || (n > 0U && param == NULL)) return SX_ERR_PARAM;

  tx[0] = op;
  for (uint8_t i = 0; i < n; i++) tx[1U + i] = param[i];
  return transferir(tx, rx, (uint16_t)(1U + n));
}

/* ===================== Etapa 1 ===================== */
sx_status_t sx1262_init(SPI_HandleTypeDef *hspi)
{
  if (hspi == NULL) return SX_ERR_PARAM;
  s_hspi = hspi;
  nss(false);                     /* arranca sin seleccionar */
  return sx1262_reset();
}

sx_status_t sx1262_reset(void)
{
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(SX_RESET_PULSO_MS);
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(1);                   /* dar tiempo a que BUSY suba antes de mirarlo */
  return esperar_libre(SX_RESET_ARRANQUE_MS);   /* BUSY baja cuando termino de arrancar */
}

/* GetStatus: se manda [0xC0, NOP]; el byte de estado vuelve junto con el NOP */
sx_status_t sx1262_get_status(uint8_t *status)
{
  uint8_t tx[2] = { OP_GET_STATUS, SX_NOP };
  uint8_t rx[2] = { 0 };

  if (status == NULL) return SX_ERR_PARAM;

  sx_status_t r = transferir(tx, rx, sizeof(tx));
  *status = rx[1];
  return r;
}

/* ReadRegister: se manda [0x1D, dir_alta, dir_baja, NOP] y despues n bytes
 * cualquiera; los datos vuelven a partir del 5to byte (rx[4] en adelante). */
sx_status_t sx1262_read_register(uint16_t addr, uint8_t *buf, uint8_t n)
{
  uint8_t tx[4U + SX_MAX_LECTURA] = { 0 };
  uint8_t rx[4U + SX_MAX_LECTURA] = { 0 };

  if (buf == NULL || n == 0U || n > SX_MAX_LECTURA) return SX_ERR_PARAM;

  tx[0] = OP_READ_REGISTER;
  tx[1] = (uint8_t)(addr >> 8);
  tx[2] = (uint8_t)(addr & 0xFFU);
  tx[3] = SX_NOP;

  sx_status_t r = transferir(tx, rx, (uint16_t)(4U + n));
  for (uint8_t i = 0; i < n; i++) buf[i] = rx[4U + i];
  return r;
}

/* WriteRegister: [0x0D, dir_alta, dir_baja, datos...] */
sx_status_t sx1262_write_register(uint16_t addr, const uint8_t *buf, uint8_t n)
{
  uint8_t tx[3U + SX_MAX_LECTURA] = { 0 };
  uint8_t rx[3U + SX_MAX_LECTURA] = { 0 };

  if (buf == NULL || n == 0U || n > SX_MAX_LECTURA) return SX_ERR_PARAM;

  tx[0] = OP_WRITE_REGISTER;
  tx[1] = (uint8_t)(addr >> 8);
  tx[2] = (uint8_t)(addr & 0xFFU);
  for (uint8_t i = 0; i < n; i++) tx[3U + i] = buf[i];
  return transferir(tx, rx, (uint16_t)(3U + n));
}

/* SetRxTxFallbackMode [0x93, modo]: a que modo vuelve el chip al terminar TX o RX */
sx_status_t sx1262_set_fallback(uint8_t modo)
{
  return comando(OP_SET_FALLBACK, &modo, 1U);
}

/* ===================== Etapa 2a ===================== */
/* SetStandby [0x80, modo]. Pasar a STBY_XOSC enciende el TCXO: BUSY queda en
 * alto mientras arranca, asi que se espera con el tope largo. */
sx_status_t sx1262_set_standby(uint8_t modo)
{
  sx_status_t r = comando(OP_SET_STANDBY, &modo, 1);
  if (r != SX_OK) return r;
  return esperar_libre(SX_CALIB_TIMEOUT_MS);
}

/* SetDio3AsTcxoCtrl [0x97, tension, demora(24 bits)].
 * Desde aca, cada vez que el chip necesite el reloj de precision, primero pone
 * DIO3 en 'tension' y espera 'demora' a que el TCXO se estabilice.
 * La demora va en pasos de 15,625 us (= 1/64 ms): pasos = us * 64 / 1000. */
sx_status_t sx1262_set_tcxo(uint8_t tension, uint32_t demora_us)
{
  uint32_t pasos = (demora_us * 64U) / 1000U;
  uint8_t  p[4];

  if (tension > SX1262_TCXO_3V3 || pasos > 0xFFFFFFU) return SX_ERR_PARAM;

  p[0] = tension;
  p[1] = (uint8_t)(pasos >> 16);
  p[2] = (uint8_t)(pasos >> 8);
  p[3] = (uint8_t)(pasos);
  return comando(OP_SET_DIO3_AS_TCXO, p, sizeof(p));
}

/* Calibrate [0x89, bloques]: osciladores RC, PLL, ADC e imagen. Tarda ~3,5 ms
 * (BUSY en alto); se espera aca para que el siguiente comando no choque. */
sx_status_t sx1262_calibrate(uint8_t bloques)
{
  sx_status_t r = comando(OP_CALIBRATE, &bloques, 1);
  if (r != SX_OK) return r;
  return esperar_libre(SX_CALIB_TIMEOUT_MS);
}

/* CalibrateImage [0x98, f1, f2]: ajusta el receptor a la banda de trabajo
 * (para 902-928 MHz: 0xE1, 0xE9). */
sx_status_t sx1262_calibrate_image(uint8_t f1, uint8_t f2)
{
  uint8_t p[2] = { f1, f2 };
  sx_status_t r = comando(OP_CALIBRATE_IMAGE, p, sizeof(p));
  if (r != SX_OK) return r;
  return esperar_libre(SX_CALIB_TIMEOUT_MS);
}

/* GetDeviceErrors: se manda [0x17, NOP, NOP, NOP]; vuelven [?, estado, err_alto, err_bajo] */
sx_status_t sx1262_get_errors(uint16_t *errores)
{
  uint8_t tx[4] = { OP_GET_DEVICE_ERRORS, SX_NOP, SX_NOP, SX_NOP };
  uint8_t rx[4] = { 0 };

  if (errores == NULL) return SX_ERR_PARAM;

  sx_status_t r = transferir(tx, rx, sizeof(tx));
  *errores = (uint16_t)(((uint16_t)rx[2] << 8) | rx[3]);
  return r;
}

/* ClearDeviceErrors [0x07, 0x00, 0x00]: borra todas las banderas de error */
sx_status_t sx1262_clear_errors(void)
{
  uint8_t p[2] = { 0x00U, 0x00U };
  return comando(OP_CLEAR_DEVICE_ERRORS, p, sizeof(p));
}

/* ===================== Etapa 2b ===================== */
/* Pasa un tiempo en us a pasos de 15,625 us (24 bits), como piden SetTx y el TCXO */
static uint32_t us_a_pasos(uint32_t us)
{
  uint32_t pasos = (us * 64U) / 1000U;
  return (pasos > 0xFFFFFFU) ? 0xFFFFFFU : pasos;
}

/* SetRegulatorMode [0x96, modo]: LDO o DC-DC. Se configura en STBY_RC. */
sx_status_t sx1262_set_regulator(uint8_t modo)
{
  return comando(OP_SET_REGULATOR, &modo, 1);
}

/* SetDio2AsRfSwitchCtrl [0x9D, 1]: el chip maneja solo la llave de antena con
 * DIO2 (en alto mientras transmite, en bajo el resto del tiempo). */
sx_status_t sx1262_set_dio2_rf_switch(bool habilitar)
{
  uint8_t p = habilitar ? 0x01U : 0x00U;
  return comando(OP_SET_DIO2_RF_SWITCH, &p, 1);
}

/* SetPacketType [0x8A, tipo]: LoRa (0x01) o FSK (0x00). Va antes que el resto. */
sx_status_t sx1262_set_packet_type(uint8_t tipo)
{
  return comando(OP_SET_PACKET_TYPE, &tipo, 1);
}

/* SetRfFrequency [0x86, f(32 bits)]: f = Hz * 2^25 / 32 MHz (el reloj del TCXO).
 * Para 915 MHz da 0x39300000. */
sx_status_t sx1262_set_rf_frequency(uint32_t hz)
{
  uint32_t f = (uint32_t)(((uint64_t)hz << 25) / 32000000ULL);
  uint8_t  p[4] = { (uint8_t)(f >> 24), (uint8_t)(f >> 16), (uint8_t)(f >> 8), (uint8_t)f };
  return comando(OP_SET_RF_FREQUENCY, p, sizeof(p));
}

/* SetPaConfig [0x95, duty, hpMax, 0x00 (=SX1262), 0x01]: como se arma el
 * amplificador de potencia. Con (0x04, 0x07) puede llegar a +22 dBm y la potencia
 * real queda dada por SetTxParams. */
sx_status_t sx1262_set_pa_config(uint8_t duty, uint8_t hp_max)
{
  uint8_t p[4] = { duty, hp_max, 0x00U, 0x01U };
  return comando(OP_SET_PA_CONFIG, p, sizeof(p));
}

/* SetTxParams [0x8E, potencia(dBm, -9..+22), rampa] */
sx_status_t sx1262_set_tx_params(int8_t dbm, uint8_t rampa)
{
  uint8_t p[2];

  if (dbm < -9 || dbm > 22) return SX_ERR_PARAM;
  p[0] = (uint8_t)dbm;
  p[1] = rampa;
  return comando(OP_SET_TX_PARAMS, p, sizeof(p));
}

/* SetBufferBaseAddress [0x8F, base_tx, base_rx]: donde arranca cada paquete en
 * la memoria de 256 bytes del chip. */
sx_status_t sx1262_set_buffer_base(uint8_t base_tx, uint8_t base_rx)
{
  uint8_t p[2] = { base_tx, base_rx };
  return comando(OP_SET_BUFFER_BASE, p, sizeof(p));
}

/* SetModulationParams (LoRa) [0x8B, SF, BW, CR, LDRO] */
sx_status_t sx1262_set_lora_modulation(uint8_t sf, uint8_t bw, uint8_t cr, bool ldro)
{
  uint8_t p[4] = { sf, bw, cr, ldro ? 0x01U : 0x00U };

  if (sf < 5U || sf > 12U) return SX_ERR_PARAM;
  return comando(OP_SET_MODULATION, p, sizeof(p));
}

/* SetPacketParams (LoRa) [0x8C, preambulo(16 bits), header, largo, CRC, IQ]
 * header 0x00 = explicito (el largo viaja en el paquete), IQ 0x00 = normal. */
sx_status_t sx1262_set_lora_packet(uint16_t preambulo, uint8_t largo, bool crc)
{
  uint8_t p[6] = { (uint8_t)(preambulo >> 8), (uint8_t)preambulo,
                   0x00U, largo, crc ? 0x01U : 0x00U, 0x00U };
  return comando(OP_SET_PACKET_PARAMS, p, sizeof(p));
}

/* SetDioIrqParams [0x08, irq(16), dio1(16), dio2(16), dio3(16)]: que eventos se
 * registran y cuales levantan el pin DIO1. DIO2 y DIO3 no se usan (antena y TCXO). */
sx_status_t sx1262_set_dio_irq(uint16_t irq_mask, uint16_t dio1_mask)
{
  uint8_t p[8] = { (uint8_t)(irq_mask >> 8),  (uint8_t)irq_mask,
                   (uint8_t)(dio1_mask >> 8), (uint8_t)dio1_mask,
                   0x00U, 0x00U, 0x00U, 0x00U };
  return comando(OP_SET_DIO_IRQ_PARAMS, p, sizeof(p));
}

/* WriteBuffer [0x0E, offset, datos...]: carga el paquete en la memoria del chip */
sx_status_t sx1262_write_buffer(uint8_t offset, const uint8_t *datos, uint8_t n)
{
  uint8_t tx[2U + SX1262_MAX_PAYLOAD] = { 0 };
  uint8_t rx[2U + SX1262_MAX_PAYLOAD] = { 0 };

  if (datos == NULL || n == 0U || n > SX1262_MAX_PAYLOAD) return SX_ERR_PARAM;

  tx[0] = OP_WRITE_BUFFER;
  tx[1] = offset;
  for (uint8_t i = 0; i < n; i++) tx[2U + i] = datos[i];
  return transferir(tx, rx, (uint16_t)(2U + n));
}

/* SetTx [0x83, timeout(24 bits)]: arranca la transmision. Si no termina en
 * 'timeout_us', el chip corta solo y levanta IRQ_TIMEOUT. 0 = sin limite. */
sx_status_t sx1262_set_tx(uint32_t timeout_us)
{
  uint32_t t = us_a_pasos(timeout_us);
  uint8_t  p[3] = { (uint8_t)(t >> 16), (uint8_t)(t >> 8), (uint8_t)t };
  return comando(OP_SET_TX, p, sizeof(p));
}

/* GetIrqStatus: se manda [0x12, NOP, NOP, NOP]; vuelven [?, estado, irq_alto, irq_bajo] */
sx_status_t sx1262_get_irq(uint16_t *irq)
{
  uint8_t tx[4] = { OP_GET_IRQ_STATUS, SX_NOP, SX_NOP, SX_NOP };
  uint8_t rx[4] = { 0 };

  if (irq == NULL) return SX_ERR_PARAM;

  sx_status_t r = transferir(tx, rx, sizeof(tx));
  *irq = (uint16_t)(((uint16_t)rx[2] << 8) | rx[3]);
  return r;
}

/* ClearIrqStatus [0x02, mascara(16)]: borra las interrupciones indicadas (y baja DIO1) */
sx_status_t sx1262_clear_irq(uint16_t mascara)
{
  uint8_t p[2] = { (uint8_t)(mascara >> 8), (uint8_t)mascara };
  return comando(OP_CLEAR_IRQ_STATUS, p, sizeof(p));
}

/* Nivel del pin DIO1: en alto = hay una interrupcion pendiente */
bool sx1262_dio1(void)
{
  return HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin) == GPIO_PIN_SET;
}

/* ===================== Etapa 2c ===================== */
/* SetRx [0x82, timeout(24 bits, en pasos de 15,625 us)]. Con 0xFFFFFF el chip
 * queda escuchando para siempre: despues de cada paquete sigue en RX solo. */
sx_status_t sx1262_set_rx(uint32_t timeout_pasos)
{
  uint8_t p[3] = { (uint8_t)(timeout_pasos >> 16), (uint8_t)(timeout_pasos >> 8),
                   (uint8_t)timeout_pasos };
  return comando(OP_SET_RX, p, sizeof(p));
}

/* GetRxBufferStatus: [0x13, NOP, NOP, NOP] -> [?, estado, largo, inicio]
 * Cuantos bytes llegaron y en que posicion de la memoria del chip quedaron. */
sx_status_t sx1262_get_rx_buffer_status(uint8_t *largo, uint8_t *inicio)
{
  uint8_t tx[4] = { OP_GET_RX_BUFFER_STATUS, SX_NOP, SX_NOP, SX_NOP };
  uint8_t rx[4] = { 0 };

  if (largo == NULL || inicio == NULL) return SX_ERR_PARAM;

  sx_status_t r = transferir(tx, rx, sizeof(tx));
  *largo  = rx[2];
  *inicio = rx[3];
  return r;
}

/* ReadBuffer: [0x1E, offset, NOP, n bytes cualquiera]; los datos vuelven desde rx[3] */
sx_status_t sx1262_read_buffer(uint8_t offset, uint8_t *datos, uint8_t n)
{
  uint8_t tx[3U + SX1262_MAX_PAYLOAD] = { 0 };
  uint8_t rx[3U + SX1262_MAX_PAYLOAD] = { 0 };

  if (datos == NULL || n == 0U || n > SX1262_MAX_PAYLOAD) return SX_ERR_PARAM;

  tx[0] = OP_READ_BUFFER;
  tx[1] = offset;
  tx[2] = SX_NOP;

  sx_status_t r = transferir(tx, rx, (uint16_t)(3U + n));
  for (uint8_t i = 0; i < n; i++) datos[i] = rx[3U + i];
  return r;
}

/* GetPacketStatus (LoRa): [0x14, NOP x4] -> [?, estado, RssiPkt, SnrPkt, SignalRssiPkt]
 *   RSSI [dBm] = -RssiPkt / 2      SNR [dB] = SnrPkt (con signo) / 4
 * El SNR se devuelve multiplicado por 4 para no usar float. */
sx_status_t sx1262_get_packet_status(int16_t *rssi_dbm, int8_t *snr_x4)
{
  uint8_t tx[5] = { OP_GET_PACKET_STATUS, SX_NOP, SX_NOP, SX_NOP, SX_NOP };
  uint8_t rx[5] = { 0 };

  if (rssi_dbm == NULL || snr_x4 == NULL) return SX_ERR_PARAM;

  sx_status_t r = transferir(tx, rx, sizeof(tx));
  *rssi_dbm = (int16_t)(-(int16_t)rx[2] / 2);
  *snr_x4   = (int8_t)rx[3];
  return r;
}

/* ===================== Etapa 4: bajo consumo ===================== */
/* SetSleep [0x84, config]. bit 2 = 1: "warm start", la configuracion (LoRa,
 * frecuencia, potencia, TCXO, IRQ) queda guardada y al despertar no hay que
 * rehacer radio_init(). La memoria de datos se pierde (se carga en cada envio).
 * Dormido consume < 1 uA (en standby RC ~0,6 mA). Mientras duerme, BUSY queda
 * en alto: el unico comando valido es despertarlo con sx1262_wakeup(). */
#define SX_SLEEP_WARM_START     0x04U

sx_status_t sx1262_set_sleep(bool warm_start)
{
  uint8_t p = warm_start ? SX_SLEEP_WARM_START : 0x00U;
  return comando(OP_SET_SLEEP, &p, 1U);
}

/* Despertar: el flanco de bajada de NSS lo saca de sleep. Se manda un GetStatus
 * sin esperar BUSY (dormido esta en alto) y despues se espera a que BUSY baje.
 * Queda en STDBY_RC, igual que despues de cada transmision. */
sx_status_t sx1262_wakeup(void)
{
  uint8_t tx[2] = { OP_GET_STATUS, SX_NOP };
  uint8_t rx[2] = { 0 };

  if (s_hspi == NULL) return SX_ERR_PARAM;

  nss(true);
  HAL_StatusTypeDef r = HAL_SPI_TransmitReceive(s_hspi, tx, rx, sizeof(tx), SX_SPI_TIMEOUT_MS);
  nss(false);
  if (r != HAL_OK) return SX_ERR_SPI;

  return esperar_libre(SX_DESPERTAR_MS);
}
