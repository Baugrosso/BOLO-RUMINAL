/**
 * @file    bolo_util.c
 * @brief   CRC-32 por hardware, medicion de VDD con el ADC y xorshift32.
 *          Se manejan los registros directo (sin modulos HAL extra), asi no hace
 *          falta tocar el .ioc. Compilado en la PC (BOLO_HOST) usa CRC por software.
 */
#include "bolo_util.h"
#include <stdbool.h>

#ifndef BOLO_HOST
#include "main.h"
#endif

/* ===================== CRC-32 ===================== */
/* Version por software (tabla de 16 entradas): mismo resultado que zlib.crc32.
 * Se usa en la PC y, en el micro, si el autotest de la unidad por hardware falla. */
static uint32_t crc32_sw(const uint8_t *datos, size_t n, uint32_t previo)
{
  static const uint32_t k_t[16] = {
    0x00000000U, 0x1DB71064U, 0x3B6E20C8U, 0x26D930ACU, 0x76DC4190U, 0x6B6B51F4U,
    0x4DB26158U, 0x5005713CU, 0xEDB88320U, 0xF00F9344U, 0xD6D6A3E8U, 0xCB61B38CU,
    0x9B64C2B0U, 0x86D3D2D4U, 0xA00AE278U, 0xBDBDF21CU };
  uint32_t crc = ~previo;
  for (size_t i = 0; i < n; i++)
  {
    crc ^= datos[i];
    crc = (crc >> 4) ^ k_t[crc & 0x0FU];
    crc = (crc >> 4) ^ k_t[crc & 0x0FU];
  }
  return ~crc;
}

#ifdef BOLO_HOST
uint32_t crc32_calc(const uint8_t *datos, size_t n, uint32_t previo)
{
  return crc32_sw(datos, n, previo);
}

bool crc32_autotest(void) { return true; }
#else
static bool s_crc_hw = true;

/* Unidad CRC del STM32L4: polinomio por defecto 0x04C11DB7 de 32 bits.
 *  REV_IN = 01 (cada byte entra con los bits invertidos) y REV_OUT = 1 -> CRC
 *  "reflejado", igual al de zlib. El XOR final se hace aca.
 *  Para encadenar, el valor previo (sin el XOR final) se carga como INIT. */
static uint32_t crc32_hw(const uint8_t *datos, size_t n, uint32_t previo)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  (void)RCC->AHB1ENR;                               /* espera a que llegue el reloj */

  /* INIT se escribe sin reflejar: el estado interno tiene los bits al reves del
   * resultado (REV_OUT), asi que el previo se invierte bit a bit (__RBIT). */
  CRC->INIT = __RBIT(~previo);
  CRC->CR   = CRC_CR_REV_IN_0 | CRC_CR_REV_OUT | CRC_CR_RESET;

  for (size_t i = 0; i < n; i++)
  {
    *(volatile uint8_t *)&CRC->DR = datos[i];       /* acceso de 8 bits: un byte */
  }

  uint32_t r = ~CRC->DR;
  RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
  return r;
}

uint32_t crc32_calc(const uint8_t *datos, size_t n, uint32_t previo)
{
  return s_crc_hw ? crc32_hw(datos, n, previo) : crc32_sw(datos, n, previo);
}

/* Valor de referencia del CRC-32: "123456789" -> 0xCBF43926 (tambien encadenado) */
bool crc32_autotest(void)
{
  static const uint8_t k_ref[9] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
  uint32_t parte = crc32_hw(k_ref, 4U, 0U);
  s_crc_hw = (crc32_hw(k_ref, 9U, 0U) == 0xCBF43926U) &&
             (crc32_hw(&k_ref[4], 5U, parte) == 0xCBF43926U);
  return s_crc_hw;
}
#endif

/* ===================== VDD ===================== */
uint8_t vdd_codigo(uint16_t mv)
{
  if (mv <= 1800U) return 0U;
  uint32_t c = ((uint32_t)mv - 1800U + 3U) / 7U;    /* redondeo a 7 mV */
  return (uint8_t)((c > 255U) ? 255U : c);
}

#ifdef BOLO_HOST
uint16_t vdd_medir_mv(void) { return 3300U; }
#else
/* Calibracion de fabrica de VREFINT: medida con VDDA = 3,0 V (RM0394 / datasheet) */
#define VREFINT_CAL_PTR     ((const uint16_t *)0x1FFF75AAUL)
#define VREFINT_CAL_MV      3000UL
#define ADC_ESPERA_MAX      20000UL     /* tope de vueltas de cada espera */

static void espera_corta(void)          /* > 20 us a 4 MHz (arranque del regulador) */
{
  for (volatile uint32_t i = 0; i < 200U; i++) { }
}

uint16_t vdd_medir_mv(void)
{
  uint32_t lectura = 0;
  uint32_t n;

  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
  (void)RCC->AHB2ENR;

  /* Reloj sincronico HCLK/1 (AHB sin divisor) y camino a VREFINT */
  ADC1_COMMON->CCR = ADC_CCR_CKMODE_0 | ADC_CCR_VREFEN;

  /* Salir de deep power-down y prender el regulador del ADC */
  ADC1->CR = 0U;
  ADC1->CR = ADC_CR_ADVREGEN;
  espera_corta();

  /* Calibracion (entrada simple) */
  ADC1->CR |= ADC_CR_ADCAL;
  for (n = 0; (ADC1->CR & ADC_CR_ADCAL) != 0U && n < ADC_ESPERA_MAX; n++) { }
  espera_corta();

  /* Habilitar */
  ADC1->ISR = ADC_ISR_ADRDY;
  ADC1->CR |= ADC_CR_ADEN;
  for (n = 0; (ADC1->ISR & ADC_ISR_ADRDY) == 0U && n < ADC_ESPERA_MAX; n++) { }

  if ((ADC1->ISR & ADC_ISR_ADRDY) != 0U)
  {
    /* Canal 0 = VREFINT, muestreo de 640,5 ciclos (>= 4 us que pide VREFINT) */
    ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP0) | ADC_SMPR1_SMP0;
    ADC1->SQR1  = 0U;                                 /* 1 conversion, canal 0 */

    ADC1->ISR = ADC_ISR_EOC;
    ADC1->CR |= ADC_CR_ADSTART;
    for (n = 0; (ADC1->ISR & ADC_ISR_EOC) == 0U && n < ADC_ESPERA_MAX; n++) { }
    if ((ADC1->ISR & ADC_ISR_EOC) != 0U) lectura = ADC1->DR;

    ADC1->CR |= ADC_CR_ADDIS;
    for (n = 0; (ADC1->CR & ADC_CR_ADEN) != 0U && n < ADC_ESPERA_MAX; n++) { }
  }

  /* Todo apagado: regulador, deep power-down, VREFINT y reloj */
  ADC1->CR = 0U;
  ADC1->CR = ADC_CR_DEEPPWD;
  ADC1_COMMON->CCR = 0U;
  RCC->AHB2ENR &= ~RCC_AHB2ENR_ADCEN;

  if (lectura == 0U) return 0U;
  uint32_t mv = (VREFINT_CAL_MV * (uint32_t)(*VREFINT_CAL_PTR) + lectura / 2U) / lectura;
  return (uint16_t)((mv > 0xFFFFU) ? 0xFFFFU : mv);
}
#endif

/* ===================== Azar ===================== */
static uint32_t s_azar = 0x2545F491U;

void azar_semilla(uint32_t extra)
{
  uint32_t s = extra ^ 0x9E3779B9U;
#ifndef BOLO_HOST
  const uint32_t *uid = (const uint32_t *)UID_BASE;
  s ^= uid[0] ^ (uid[1] << 7) ^ (uid[2] << 13);
#endif
  s_azar = (s != 0U) ? s : 0x2545F491U;
}

static uint32_t azar_siguiente(void)
{
  uint32_t x = s_azar;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  s_azar = x;
  return x;
}

uint32_t azar_rango(uint32_t min, uint32_t max)
{
  if (max <= min) return min;
  return min + (azar_siguiente() % (max - min + 1U));
}
