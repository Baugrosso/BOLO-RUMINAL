/**
 * @file    drv_sht31.c
 * @brief   Implementacion del driver del SHT31.
 */
#include "drv_sht31.h"
#include <stddef.h>   /* NULL */

/* ===================== Comandos (privado) ===================== */
#define CMD_SOFT_RESET        0x30A2U
#define CMD_LEER_ESTADO       0xF32DU
#define CMD_MEDIR_ALTA_NOCS   0x2400U   /* single shot, alta repetibilidad, sin clock stretching */

#define SHT_TIMEOUT_MS        10U
#define SHT_T_RESET_MS        2U        /* datasheet: max 1,5 ms */

/* ===================== Variables privadas ===================== */
static I2C_HandleTypeDef *s_hi2c = NULL;
static uint16_t s_addr   = 0;           /* direccion ya corrida para la HAL */
static uint16_t s_estado = 0;

/* ===================== Funciones privadas ===================== */
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
  /* CRC-8 del datasheet: polinomio 0x31, valor inicial 0xFF */
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
    {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x31U) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static sht_status_t write_cmd(uint16_t cmd)
{
  uint8_t b[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFFU) };   /* alto primero */

  if (s_hi2c == NULL) return SHT_ERR_I2C;
  if (HAL_I2C_Master_Transmit(s_hi2c, s_addr, b, 2, SHT_TIMEOUT_MS) != HAL_OK)
  {
    return SHT_ERR_I2C;
  }
  return SHT_OK;
}

static sht_status_t read_bytes(uint8_t *buf, uint16_t len)
{
  if (s_hi2c == NULL) return SHT_ERR_I2C;
  if (HAL_I2C_Master_Receive(s_hi2c, s_addr, buf, len, SHT_TIMEOUT_MS) != HAL_OK)
  {
    return SHT_ERR_I2C;
  }
  return SHT_OK;
}

/* ===================== Funciones publicas ===================== */
sht_status_t sht31_init(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
  uint8_t b[3];

  if (hi2c == NULL) return SHT_ERR_PARAM;
  s_hi2c = hi2c;
  s_addr = (uint16_t)(addr7 << 1);

  /* 1. Reset: estado conocido */
  if (write_cmd(CMD_SOFT_RESET) != SHT_OK) return SHT_ERR_I2C;
  HAL_Delay(SHT_T_RESET_MS);

  /* 2. Leer el registro de estado: confirma que responde y que el CRC anda */
  if (write_cmd(CMD_LEER_ESTADO) != SHT_OK) return SHT_ERR_I2C;
  if (read_bytes(b, 3) != SHT_OK)          return SHT_ERR_I2C;
  if (crc8(b, 2) != b[2])                  return SHT_ERR_CRC;

  s_estado = (uint16_t)((b[0] << 8) | b[1]);
  return SHT_OK;
}

sht_status_t sht31_start(void)
{
  return write_cmd(CMD_MEDIR_ALTA_NOCS);
}

sht_status_t sht31_read(int16_t *temp_centi, uint16_t *hum_centi)
{
  uint8_t b[6];

  if (temp_centi == NULL || hum_centi == NULL) return SHT_ERR_PARAM;

  /* Si la medicion no termino, el sensor no responde (NACK) y esto da error */
  if (read_bytes(b, 6) != SHT_OK) return SHT_ERR_I2C;

  if (crc8(&b[0], 2) != b[2] || crc8(&b[3], 2) != b[5]) return SHT_ERR_CRC;

  uint32_t raw_t = ((uint32_t)b[0] << 8) | b[1];
  uint32_t raw_h = ((uint32_t)b[3] << 8) | b[4];

  /* Formulas del datasheet en enteros (centesimas), con redondeo */
  *temp_centi = (int16_t)(-4500 + (int32_t)((17500U * raw_t + 32767U) / 65535U));
  *hum_centi  = (uint16_t)((10000U * raw_h + 32767U) / 65535U);

  return SHT_OK;
}

uint16_t sht31_get_status(void)
{
  return s_estado;
}
