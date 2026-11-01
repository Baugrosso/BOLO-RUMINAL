/**
 * @file    sensor_temp.h
 * @brief   Interfaz generica de temperatura.
 *          La aplicacion usa estas funciones y nunca nombra al chip.
 *          En el bolo final apunta al TMP117 cambiando solo este archivo y el driver.
 */
#ifndef SENSOR_TEMP_H
#define SENSOR_TEMP_H

#include <stdbool.h>
#include "bolo_config.h"
#include "drv_sht31.h"

#define TEMP_NOMBRE          "SHT31"
#define TEMP_T_MEDICION_MS   SHT31_T_MEDICION_MS   /* lo que tarda una medicion (peor caso) */

static inline bool temp_init(I2C_HandleTypeDef *hi2c)              { return sht31_init(hi2c, TEMP_I2C_ADDR) == SHT_OK; }
static inline bool temp_start(void)                                 { return sht31_start() == SHT_OK; }   /* dispara una medicion */
static inline bool temp_read(int16_t *t_centi, uint16_t *h_centi)   { return sht31_read(t_centi, h_centi) == SHT_OK; }

#endif /* SENSOR_TEMP_H */
