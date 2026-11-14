/**
 * @file    drv_mpu6050.h
 * @brief   Driver del MPU-6050 (GY-521): solo acelerometro, en low-power cycle.
 *
 *  Configuracion del requisito (T5): acelerometro a 5 Hz en low-power, +-2 g,
 *  giroscopo y sensor de temperatura apagados. El chip se despierta solo 5 veces
 *  por segundo, toma una muestra y avisa por INT (plan A). Leer los datos borra
 *  el aviso.
 *    PWR_MGMT_1   = 0x28  CYCLE = 1, TEMP_DIS = 1, reloj interno
 *    PWR_MGMT_2   = 0x47  LP_WAKE_CTRL = 01 (5 Hz), giroscopo X/Y/Z en standby
 *    ACCEL_CONFIG = 0x00  +-2 g
 *    INT_PIN_CFG  = 0x10  INT_RD_CLEAR: cualquier lectura borra el aviso
 *    INT_ENABLE   = 0x01  DATA_RDY_EN
 *  (Plan B, la FIFO en low-power, queda a verificar en el banco.)
 */
#ifndef DRV_MPU6050_H
#define DRV_MPU6050_H

#include "main.h"          /* tipos de la HAL (I2C_HandleTypeDef) */
#include "bolo_types.h"    /* acc_raw_t */

/* Sensibilidad en el rango de +-2 g: 16384 cuentas = 1 g */
#define MPU6050_LSB_POR_G   16384

typedef enum
{
  MPU_OK = 0,
  MPU_ERR_I2C,     /* el sensor no respondio en el bus      */
  MPU_ERR_ID,      /* respondio, pero no es un MPU-6050     */
  MPU_ERR_PARAM    /* parametro invalido                    */
} mpu_status_t;

mpu_status_t mpu6050_init(I2C_HandleTypeDef *hi2c);   /* reset + low-power cycle a 5 Hz */
mpu_status_t mpu6050_read_acc(acc_raw_t *out);        /* tambien borra el aviso de INT  */
uint8_t      mpu6050_get_id(void);

#endif /* DRV_MPU6050_H */
