/**
 * @file    drv_sht31.h
 * @brief   Driver del SHT31 (temperatura y humedad), modo single shot.
 */
#ifndef DRV_SHT31_H
#define DRV_SHT31_H

#include "main.h"
#include <stdint.h>

/* Tiempo maximo de una medicion en alta repetibilidad (datasheet) */
#define SHT31_T_MEDICION_MS   15U

typedef enum
{
  SHT_OK = 0,
  SHT_ERR_I2C,     /* no respondio (o se leyo antes de terminar la medicion) */
  SHT_ERR_CRC,     /* respondio, pero los datos llegaron corruptos           */
  SHT_ERR_PARAM    /* parametro invalido                                     */
} sht_status_t;

sht_status_t sht31_init(I2C_HandleTypeDef *hi2c, uint8_t addr7);
sht_status_t sht31_start(void);
sht_status_t sht31_read(int16_t *temp_centi, uint16_t *hum_centi);
uint16_t     sht31_get_status(void);

#endif /* DRV_SHT31_H */
