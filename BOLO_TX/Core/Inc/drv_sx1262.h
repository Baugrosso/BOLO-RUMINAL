/**
 * @file    drv_sx1262.h
 * @brief   Driver minimo del transceptor LoRa SX1262 (Semtech) por SPI.
 *          Etapa 1 : reset, lectura de estado y de registros.
 *          Etapa 2a: modos de standby, TCXO, calibracion y errores internos.
 *          Etapa 2b: configuracion LoRa y transmision.
 *          Etapa 2c: recepcion.
 *          Los pines salen de los User Labels de CubeMX (main.h):
 *          LORA_NSS, LORA_RST, LORA_BUSY (y mas adelante LORA_DIO1, LORA_RFSW).
 */
#ifndef DRV_SX1262_H
#define DRV_SX1262_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
  SX_OK = 0,
  SX_ERR_SPI,       /* fallo la transferencia SPI (timeout de la HAL)     */
  SX_ERR_BUSY,      /* el chip quedo ocupado (BUSY en alto) demasiado     */
  SX_ERR_PARAM      /* parametro invalido o driver sin inicializar        */
} sx_status_t;

/* ---- Registros que usamos ---- */
#define SX1262_REG_LORA_SYNC_WORD   0x0740U   /* 2 bytes; valor de fabrica 0x14 0x24 */
#define SX1262_REG_OCP              0x08E7U   /* limite de corriente del PA (2,5 mA por paso) */
#define SX1262_OCP_140MA            0x38U     /* lo que pide +22 dBm                         */

/* ---- Byte de estado (GetStatus): bits 6:4 = modo, bits 3:1 = resultado del comando ---- */
#define SX1262_MODO(status)         (((status) >> 4) & 0x07U)
#define SX1262_CMD(status)          (((status) >> 1) & 0x07U)
#define SX1262_MODO_STBY_RC         2U        /* standby con el oscilador RC interno */
#define SX1262_MODO_STBY_XOSC       3U        /* standby con el TCXO encendido       */
#define SX1262_MODO_FS              4U
#define SX1262_MODO_RX              5U
#define SX1262_MODO_TX              6U

/* ---- SetStandby ---- */
#define SX1262_STDBY_RC             0x00U     /* reloj interno RC de 13 MHz (menor consumo) */
#define SX1262_STDBY_XOSC           0x01U     /* reloj de precision (TCXO) ya encendido     */

/* ---- SetDio3AsTcxoCtrl: tension con la que DIO3 alimenta al TCXO ---- */
#define SX1262_TCXO_1V6             0x00U
#define SX1262_TCXO_1V7             0x01U
#define SX1262_TCXO_1V8             0x02U
#define SX1262_TCXO_2V2             0x03U
#define SX1262_TCXO_2V4             0x04U
#define SX1262_TCXO_2V7             0x05U
#define SX1262_TCXO_3V0             0x06U
#define SX1262_TCXO_3V3             0x07U

/* ---- Calibrate: que bloques calibrar (0x7F = todos) ---- */
#define SX1262_CALIB_TODO           0x7FU

/* ---- CalibrateImage: banda de trabajo (valores del datasheet) ---- */
#define SX1262_IMG_902_928_F1       0xE1U
#define SX1262_IMG_902_928_F2       0xE9U

/* ---- GetDeviceErrors: un bit por cada falla interna ---- */
#define SX1262_ERR_RC64K_CALIB      (1U << 0)
#define SX1262_ERR_RC13M_CALIB      (1U << 1)
#define SX1262_ERR_PLL_CALIB        (1U << 2)
#define SX1262_ERR_ADC_CALIB        (1U << 3)
#define SX1262_ERR_IMG_CALIB        (1U << 4)
#define SX1262_ERR_XOSC_START       (1U << 5)   /* el oscilador (TCXO) no arranco */
#define SX1262_ERR_PLL_LOCK         (1U << 6)
#define SX1262_ERR_PA_RAMP          (1U << 8)

/* ---- SetRegulatorMode ---- */
#define SX1262_REGULADOR_LDO        0x00U
#define SX1262_REGULADOR_DCDC       0x01U     /* consume menos; el modulo trae la bobina */

/* ---- SetPacketType ---- */
#define SX1262_PAQUETE_LORA         0x01U

/* ---- SetModulationParams (LoRa): ancho de banda y tasa de codigo ---- */
#define SX1262_BW_62_5              0x03U
#define SX1262_BW_125               0x04U
#define SX1262_BW_250               0x05U
#define SX1262_BW_500               0x06U
#define SX1262_CR_4_5               0x01U
#define SX1262_CR_4_6               0x02U
#define SX1262_CR_4_7               0x03U
#define SX1262_CR_4_8               0x04U

/* ---- SetTxParams: rampa de subida del amplificador ---- */
#define SX1262_RAMPA_200US          0x04U

/* ---- Interrupciones (GetIrqStatus / SetDioIrqParams) ---- */
#define SX1262_IRQ_TX_DONE          (1U << 0)
#define SX1262_IRQ_RX_DONE          (1U << 1)
#define SX1262_IRQ_HEADER_ERR       (1U << 5)
#define SX1262_IRQ_CRC_ERR          (1U << 6)
#define SX1262_IRQ_TIMEOUT          (1U << 9)
#define SX1262_IRQ_TODAS            0x03FFU

#define SX1262_MAX_PAYLOAD          66U       /* lo maximo que dejamos mandar de una vez (SF9) */

/* ---- Etapa 1 ---- */
sx_status_t sx1262_init(SPI_HandleTypeDef *hspi);          /* guarda el SPI y resetea */
sx_status_t sx1262_reset(void);                            /* pulso en RST y espera   */
sx_status_t sx1262_get_status(uint8_t *status);            /* comando GetStatus       */
sx_status_t sx1262_read_register(uint16_t addr, uint8_t *buf, uint8_t n);

/* ---- Etapa 2a ---- */
sx_status_t sx1262_set_standby(uint8_t modo);
sx_status_t sx1262_set_tcxo(uint8_t tension, uint32_t demora_us);
sx_status_t sx1262_calibrate(uint8_t bloques);
sx_status_t sx1262_calibrate_image(uint8_t f1, uint8_t f2);
sx_status_t sx1262_get_errors(uint16_t *errores);
sx_status_t sx1262_clear_errors(void);

/* ---- Etapa 2b ---- */
sx_status_t sx1262_set_regulator(uint8_t modo);
sx_status_t sx1262_set_dio2_rf_switch(bool habilitar);
sx_status_t sx1262_set_packet_type(uint8_t tipo);
sx_status_t sx1262_set_rf_frequency(uint32_t hz);
sx_status_t sx1262_set_pa_config(uint8_t duty, uint8_t hp_max);
sx_status_t sx1262_set_tx_params(int8_t dbm, uint8_t rampa);
sx_status_t sx1262_set_buffer_base(uint8_t base_tx, uint8_t base_rx);
sx_status_t sx1262_set_lora_modulation(uint8_t sf, uint8_t bw, uint8_t cr, bool ldro);
sx_status_t sx1262_set_lora_packet(uint16_t preambulo, uint8_t largo, bool crc);
sx_status_t sx1262_set_dio_irq(uint16_t irq_mask, uint16_t dio1_mask);
sx_status_t sx1262_write_buffer(uint8_t offset, const uint8_t *datos, uint8_t n);
sx_status_t sx1262_set_tx(uint32_t timeout_us);
sx_status_t sx1262_get_irq(uint16_t *irq);
sx_status_t sx1262_clear_irq(uint16_t mascara);
bool        sx1262_dio1(void);                             /* nivel del pin DIO1 */

/* ---- Etapa 2c ---- */
#define SX1262_RX_CONTINUO          0xFFFFFFU /* SetRx sin timeout: escucha siempre */

sx_status_t sx1262_set_rx(uint32_t timeout_pasos);
sx_status_t sx1262_get_rx_buffer_status(uint8_t *largo, uint8_t *inicio);
sx_status_t sx1262_read_buffer(uint8_t offset, uint8_t *datos, uint8_t n);
sx_status_t sx1262_get_packet_status(int16_t *rssi_dbm, int8_t *snr_x4);

/* ---- Escritura de registros y modo de reposo entre operaciones ---- */
#define SX1262_FALLBACK_STDBY_RC    0x20U     /* despues de TX/RX vuelve a STDBY_RC (TCXO apagado)   */
#define SX1262_FALLBACK_STDBY_XOSC  0x30U     /* vuelve a STDBY_XOSC: el TCXO queda prendido y la
                                               * proxima operacion arranca sin esperar 5 ms         */
sx_status_t sx1262_write_register(uint16_t addr, const uint8_t *buf, uint8_t n);
sx_status_t sx1262_set_fallback(uint8_t modo);

/* ---- Etapa 4: bajo consumo ---- */
sx_status_t sx1262_set_sleep(bool warm_start);   /* dormir (< 1 uA)                  */
sx_status_t sx1262_wakeup(void);                 /* despertar: queda en STDBY_RC     */

#endif /* DRV_SX1262_H */
