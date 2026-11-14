/**
 * @file    drv_mpu6050.c
 * @brief   Implementacion del driver del MPU-6050 en low-power cycle (5 Hz).
 */
#include "drv_mpu6050.h"
#include <stddef.h>   /* NULL */

/* ===================== Direccion y registros (privado) ===================== */
#define MPU_ADDR            (0x68U << 1)   /* AD0 a GND. La HAL usa la direccion corrida */
#define MPU_ID_ESPERADO     0x68U

#define REG_ACCEL_CONFIG    0x1CU
#define REG_INT_PIN_CFG     0x37U
#define REG_INT_ENABLE      0x38U
#define REG_ACCEL_XOUT_H    0x3BU   /* primer byte de los datos */
#define REG_PWR_MGMT_1      0x6BU
#define REG_PWR_MGMT_2      0x6CU
#define REG_WHO_AM_I        0x75U

/* Valores del requisito (T5) */
#define PWR1_DEVICE_RESET   0x80U
#define PWR1_LP_CYCLE       0x28U   /* CYCLE (bit 5) + TEMP_DIS (bit 3), reloj interno */
#define PWR2_LP_5HZ_SIN_GYR 0x47U   /* LP_WAKE_CTRL = 01 (5 Hz) + STBY_XG/YG/ZG        */
#define ACC_RANGO_2G        0x00U
#define INT_CFG_RD_CLEAR    0x10U   /* activo en alto, push-pull, pulso de 50 us       */
#define INT_EN_DATA_RDY     0x01U

/* Tiempos */
#define MPU_TIMEOUT_MS      10U     /* nunca esperar mas que esto al bus */
#define MPU_T_RESET_MS      100U    /* despues de un reset               */

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
mpu_status_t mpu6050_init(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL) return MPU_ERR_PARAM;
  s_hi2c = hi2c;

  /* 1. Identificar: ¿hay un MPU-6050 del otro lado? */
  s_id = 0;                                   /* 0 = no respondio (no queda el ID viejo) */
  if (read_regs(REG_WHO_AM_I, &s_id, 1) != MPU_OK) { s_id = 0; return MPU_ERR_I2C; }
  if (s_id != MPU_ID_ESPERADO)                      return MPU_ERR_ID;

  /* 2. Reset: arrancar siempre desde un estado conocido */
  if (write_reg(REG_PWR_MGMT_1, PWR1_DEVICE_RESET) != MPU_OK) return MPU_ERR_I2C;
  HAL_Delay(MPU_T_RESET_MS);

  /* 3. Rango y aviso por INT primero; al final el modo de energia, que es lo que
   *    arranca el ciclo de 5 Hz */
  if (write_reg(REG_ACCEL_CONFIG, ACC_RANGO_2G)        != MPU_OK ||
      write_reg(REG_INT_PIN_CFG,  INT_CFG_RD_CLEAR)    != MPU_OK ||
      write_reg(REG_INT_ENABLE,   INT_EN_DATA_RDY)     != MPU_OK ||
      write_reg(REG_PWR_MGMT_2,   PWR2_LP_5HZ_SIN_GYR) != MPU_OK ||
      write_reg(REG_PWR_MGMT_1,   PWR1_LP_CYCLE)       != MPU_OK)
  {
    return MPU_ERR_I2C;
  }

  /* 4. Verificar que el modo quedo escrito (el bus puede aceptar y el chip no) */
  uint8_t p1 = 0, p2 = 0;
  if (read_regs(REG_PWR_MGMT_1, &p1, 1) != MPU_OK ||
      read_regs(REG_PWR_MGMT_2, &p2, 1) != MPU_OK)  return MPU_ERR_I2C;
  if (p1 != PWR1_LP_CYCLE || p2 != PWR2_LP_5HZ_SIN_GYR) return MPU_ERR_I2C;

  return MPU_OK;
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
