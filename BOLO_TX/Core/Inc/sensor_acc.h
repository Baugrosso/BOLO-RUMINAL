/**
 * @file    sensor_acc.h
 * @brief   Interfaz generica de acelerometro.
 *          La aplicacion usa estas funciones y nunca nombra al chip.
 *          Para cambiar de sensor (ej. LIS2DW12 en el bolo final) se cambia
 *          solo este archivo y el driver correspondiente.
 */
#ifndef SENSOR_ACC_H
#define SENSOR_ACC_H

#include <stdbool.h>
#include "bolo_config.h"
#include "bolo_types.h"
#include "drv_mpu6050.h"

#define ACC_NOMBRE          "MPU-6050"
#define ACC_CUENTAS_POR_G   MPU6050_LSB_POR_G

/* static inline: el compilador pega el cuerpo donde se llama; la capa no agrega costo */
static inline bool    acc_init(I2C_HandleTypeDef *hi2c) { return mpu6050_init(hi2c, ACC_FS_HZ) == MPU_OK; }
static inline bool    acc_wake(void)                    { return mpu6050_wake()  == MPU_OK; }
static inline bool    acc_sleep(void)                   { return mpu6050_sleep() == MPU_OK; }
static inline bool    acc_read(acc_raw_t *m)            { return mpu6050_read_acc(m) == MPU_OK; }
static inline uint8_t acc_id(void)                      { return mpu6050_get_id(); }

#endif /* SENSOR_ACC_H */
