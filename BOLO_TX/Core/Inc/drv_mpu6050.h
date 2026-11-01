/**
 * @file    drv_mpu6050.h
 * @brief   Driver del MPU-6050 (GY-521): solo acelerometro, orientado a bajo consumo.
 */
#ifndef DRV_MPU6050_H
#define DRV_MPU6050_H

#include "main.h"          /* tipos de la HAL (I2C_HandleTypeDef) */
#include "bolo_types.h"    /* acc_raw_t */

/* Sensibilidad en el rango de +-2 g: 16384 cuentas = 1 g */
#define MPU6050_LSB_POR_G   16384.0f

/* Resultado de cada operacion del driver */
typedef enum
{
  MPU_OK = 0,
  MPU_ERR_I2C,     /* el sensor no respondio en el bus      */
  MPU_ERR_ID,      /* respondio, pero no es un MPU-6050     */
  MPU_ERR_PARAM    /* parametro invalido                    */
} mpu_status_t;

mpu_status_t mpu6050_init(I2C_HandleTypeDef *hi2c, uint16_t fs_hz);
mpu_status_t mpu6050_wake(void);
mpu_status_t mpu6050_sleep(void);
mpu_status_t mpu6050_read_acc(acc_raw_t *out);
uint8_t      mpu6050_get_id(void);

#endif /* DRV_MPU6050_H */
