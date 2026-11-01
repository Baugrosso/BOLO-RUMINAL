/**
 * @file    drv_mpu6050.c
 * @brief   Implementacion del driver del MPU-6050.
 */
#include "drv_mpu6050.h"
#include <stddef.h>   /* NULL */

/* ===================== Direccion y registros (privado) ===================== */
#define MPU_ADDR            (0x68U << 1)   /* AD0 a GND. La HAL usa la direccion corrida */
#define MPU_ID_ESPERADO     0x68U

#define REG_SMPLRT_DIV      0x19U   /* divisor de frecuencia de muestreo */
#define REG_CONFIG          0x1AU   /* filtro pasa-bajos (DLPF)           */
#define REG_GYRO_CONFIG     0x1BU
#define REG_ACCEL_CONFIG    0x1CU   /* rango del acelerometro             */
#define REG_ACCEL_XOUT_H    0x3BU   /* primer byte de los datos           */
#define REG_PWR_MGMT_1      0x6BU   /* energia: sleep, reset, reloj       */
#define REG_PWR_MGMT_2      0x6CU   /* energia: apagar ejes               */
#define REG_WHO_AM_I        0x75U

/* Bits de PWR_MGMT_1 */
#define PWR1_DEVICE_RESET   0x80U   /* bit 7 */
#define PWR1_SLEEP          0x40U   /* bit 6 */
#define PWR1_TEMP_DIS       0x08U   /* bit 3: apaga el sensor de temperatura interno */
#define PWR1_CLK_INTERNO    0x00U   /* bits 2-0: oscilador interno de 8 MHz          */

/* Bits de PWR_MGMT_2 */
#define PWR2_GYRO_STBY      0x07U   /* bits 2-0: giroscopo X, Y y Z apagados */

#define DLPF_ACC_10HZ       0x05U   /* filtro pasa-bajos del acelerometro: 10 Hz */
#define ACC_RANGO_2G        0x00U   /* +-2 g */
#define DLPF_FREQ_BASE_HZ   1000U   /* con el DLPF activo, el sensor trabaja a 1 kHz */

/* Tiempos */
#define MPU_TIMEOUT_MS      10U     /* nunca esperar mas que esto al bus  */
#define MPU_T_RESET_MS      100U    /* despues de un reset                */
#define MPU_T_WAKE_MS       50U     /* despues de despertar               */

/* ===================== Variables privadas ===================== */
static I2C_HandleTypeDef *s_hi2c = NULL;
static uint8_t s_id = 0;

/* ===================== Funciones privadas ===================== */
static mpu_status_t write_reg(uint8_t reg, uint8_t val)
{
  if (s_hi2c == NULL) return MPU_ERR_I2C;
  if (HAL_I2C_Mem_Write(s_hi2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                        &val, 1, MPU_TIMEOUT_MS) != HAL_OK)
  {
    return MPU_ERR_I2C;
  }
  return MPU_OK;
}

static mpu_status_t read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (s_hi2c == NULL) return MPU_ERR_I2C;
  if (HAL_I2C_Mem_Read(s_hi2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                       buf, len, MPU_TIMEOUT_MS) != HAL_OK)
  {
    return MPU_ERR_I2C;
  }
  return MPU_OK;
}

/* ===================== Funciones publicas ===================== */
mpu_status_t mpu6050_init(I2C_HandleTypeDef *hi2c, uint16_t fs_hz)
{
  if (hi2c == NULL || fs_hz == 0U || fs_hz > DLPF_FREQ_BASE_HZ) return MPU_ERR_PARAM;
  s_hi2c = hi2c;

  /* 1. Identificar: ¿hay un MPU-6050 del otro lado? */
  s_id = 0;                                   /* 0 = no respondio (no queda el ID viejo) */
  if (read_regs(REG_WHO_AM_I, &s_id, 1) != MPU_OK) { s_id = 0; return MPU_ERR_I2C; }
  if (s_id != MPU_ID_ESPERADO)                      return MPU_ERR_ID;

  /* 2. Reset: arrancar siempre desde un estado conocido */
  if (write_reg(REG_PWR_MGMT_1, PWR1_DEVICE_RESET) != MPU_OK) return MPU_ERR_I2C;
  HAL_Delay(MPU_T_RESET_MS);

  /* 3. Configurar */
  uint8_t divisor = (uint8_t)((DLPF_FREQ_BASE_HZ / fs_hz) - 1U);   /* 25 Hz -> 39 */

  if (write_reg(REG_PWR_MGMT_1,   PWR1_TEMP_DIS | PWR1_CLK_INTERNO) != MPU_OK ||  /* despierto   */
      write_reg(REG_PWR_MGMT_2,   PWR2_GYRO_STBY)                   != MPU_OK ||  /* sin giro    */
      write_reg(REG_CONFIG,       DLPF_ACC_10HZ)                    != MPU_OK ||  /* filtro      */
      write_reg(REG_SMPLRT_DIV,   divisor)                          != MPU_OK ||  /* frecuencia  */
      write_reg(REG_ACCEL_CONFIG, ACC_RANGO_2G)                     != MPU_OK)    /* rango       */
  {
    return MPU_ERR_I2C;
  }

  /* 4. Queda dormido hasta que la aplicacion lo necesite */
  return mpu6050_sleep();
}

mpu_status_t mpu6050_wake(void)
{
  if (write_reg(REG_PWR_MGMT_1, PWR1_TEMP_DIS | PWR1_CLK_INTERNO) != MPU_OK) return MPU_ERR_I2C;
  HAL_Delay(MPU_T_WAKE_MS);   /* dejar que el sensor y el filtro se estabilicen */
  return MPU_OK;
}

mpu_status_t mpu6050_sleep(void)
{
  return write_reg(REG_PWR_MGMT_1, PWR1_SLEEP | PWR1_TEMP_DIS | PWR1_CLK_INTERNO);
}

mpu_status_t mpu6050_read_acc(acc_raw_t *out)
{
  uint8_t b[6];

  if (out == NULL) return MPU_ERR_PARAM;
  if (read_regs(REG_ACCEL_XOUT_H, b, sizeof(b)) != MPU_OK) return MPU_ERR_I2C;

  /* Cada eje son 2 bytes: primero el alto (H), despues el bajo (L) */
  out->x = (int16_t)((b[0] << 8) | b[1]);
  out->y = (int16_t)((b[2] << 8) | b[3]);
  out->z = (int16_t)((b[4] << 8) | b[5]);
  return MPU_OK;
}

uint8_t mpu6050_get_id(void)
{
  return s_id;
}
