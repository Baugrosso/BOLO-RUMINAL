/**
 * @file    salto.c
 * @brief   Tabla de salto pseudoaleatoria. La genera tools/generar_tabla_salto.py
 *          (semilla 0xB010, sin canales vecinos seguidos): la gateway usa la misma.
 */
#include "salto.h"

static const uint8_t k_tabla[SALTO_N_CANALES] =
{
  23,  7, 60, 29,  1,  5,  0, 31, 57, 50, 25, 17, 11, 51, 44, 62,
  28, 13, 30, 46, 22, 38, 58, 63, 54, 21, 52, 27, 59,  6, 14,  8,
  43, 24, 26,  9, 40, 53, 39, 45,  4, 15, 56, 33, 61, 32, 34, 36,
  12, 16, 19, 49,  3, 20, 42, 55, 47, 41, 37, 18, 10,  2, 48, 35,
};

uint8_t salto_canal(uint8_t contacto, uint16_t pos)
{
  return k_tabla[(32U * (uint32_t)contacto + pos) % SALTO_N_CANALES];
}

uint32_t salto_hz(uint8_t canal)
{
  return SALTO_BASE_HZ + SALTO_PASO_HZ * (uint32_t)(canal % SALTO_N_CANALES);
}
