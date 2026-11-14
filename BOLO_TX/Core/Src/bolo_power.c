/**
 * @file    bolo_power.c
 * @brief   STOP2 por eventos: tick del RTC, INT del MPU-6050 (PB1) y LPTIM1.
 *
 * Que es STOP2: se apagan los relojes rapidos (MSI) y casi todo el chip, pero se
 * conservan la RAM (los 64 KB, incluido el almacen de datos) y los registros de los
 * perifericos. Siguen vivos el LSE (RTC y LPTIM1) y el LSI (IWDG). Al despertar el
 * micro sigue en la linea siguiente al WFI, con el MSI a 4 MHz como antes.
 *
 * El pin PB1 y el LPTIM1 se configuran aca, a mano, y no en CubeMX: asi regenerar
 * el codigo desde el .ioc no los pisa. Si algun dia se pasan al .ioc (PB1 como
 * GPIO_EXTI1 con pull-down y flanco de subida; LPTIM1 con reloj LSE), hay que borrar
 * de aca EXTI1_IRQHandler / LPTIM1_IRQHandler para que no queden duplicados.
 */
#include "bolo_power.h"
#include "bolo_config.h"
#include "bolo_reloj.h"
#include <stdio.h>

#define UART_VACIADO_MS     20U       /* tope para esperar que salga el ultimo byte */
#define LSE_HZ              32768U

static UART_HandleTypeDef *s_huart      = NULL;
static volatile uint32_t   s_mpu_avisos = 0U;
static volatile bool       s_lptim_fin  = false;

/* ===================== Interrupciones ===================== */
/* INT del MPU-6050 (pulso de 50 us por cada muestra nueva) */
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(BOLO_MPU_INT_PIN);
}

/* La HAL la llama para cada linea EXTI (tambien la del DIO1 de la radio, que se
 * lee por nivel y no necesita aviso). */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BOLO_MPU_INT_PIN) s_mpu_avisos++;
}

void LPTIM1_IRQHandler(void)
{
  if ((LPTIM1->ISR & LPTIM_ISR_ARRM) != 0U)
  {
    LPTIM1->ICR = LPTIM_ICR_ARRMCF;
    LPTIM1->CR  = 0U;                    /* apagado hasta el proximo disparo */
    s_lptim_fin = true;
  }
}

/* ===================== Auxiliares ===================== */
/* Espera a que el UART termine de sacar el ultimo byte: en STOP2 el reloj del
 * UART se apaga y lo que estaba saliendo llega cortado al PuTTY. */
static void uart_vaciar(void)
{
  (void)fflush(stdout);
  if (s_huart == NULL) return;

  uint32_t t0 = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC) == RESET)
  {
    if ((HAL_GetTick() - t0) > UART_VACIADO_MS) break;
  }
}

static bool hay_pendiente(void)
{
  return (s_mpu_avisos != 0U) || s_lptim_fin || reloj_tick_pendiente();
}

/* ===================== Publicas ===================== */
void power_init(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef g = {0};

  s_huart = huart;

#if BOLO_DBG_EN_STOP
  HAL_DBGMCU_EnableDBGStopMode();     /* el debugger sobrevive al STOP2 (consume mas) */
#else
  HAL_DBGMCU_DisableDBGStopMode();    /* consumo real                                  */
#endif
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);  /* despierta con el MSI */

  /* INT del MPU-6050 -> PB1 (D6): entrada con pull-down, interrupcion por flanco de
   * subida (el MPU da un pulso en alto, push-pull, por cada muestra) */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin  = BOLO_MPU_INT_PIN;
  g.Mode = GPIO_MODE_IT_RISING;
  g.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BOLO_MPU_INT_PORT, &g);
  HAL_NVIC_SetPriority(BOLO_MPU_INT_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(BOLO_MPU_INT_IRQn);

  /* LPTIM1 con reloj LSE: es el unico timer que sigue contando en STOP2 */
  RCC->CCIPR    = (RCC->CCIPR & ~RCC_CCIPR_LPTIM1SEL) | RCC_CCIPR_LPTIM1SEL;  /* 11 = LSE */
  RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
  (void)RCC->APB1ENR1;
  LPTIM1->CR   = 0U;
  LPTIM1->CFGR = 0U;                  /* prescaler /1, reloj interno (LSE)            */
  LPTIM1->IER  = LPTIM_IER_ARRMIE;    /* IER solo se puede escribir con ENABLE = 0    */
  EXTI->IMR2  |= EXTI_IMR2_IM32;      /* linea 32 = LPTIM1: despierta del STOP2       */
  HAL_NVIC_SetPriority(LPTIM1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
}

void power_lptim_iniciar(uint32_t ms)
{
  uint32_t ticks = (ms * LSE_HZ + 999U) / 1000U;     /* 16 ms -> 525 cuentas */
  if (ticks < 2U)      ticks = 2U;
  if (ticks > 0xFFFFU) ticks = 0xFFFFU;

  s_lptim_fin = false;
  LPTIM1->CR  = 0U;
  LPTIM1->CR  = LPTIM_CR_ENABLE;
  LPTIM1->ICR = 0x7FU;                               /* borra banderas viejas       */
  LPTIM1->ARR = ticks;
  for (uint32_t n = 0; (LPTIM1->ISR & LPTIM_ISR_ARROK) == 0U && n < 2000U; n++) { }
  LPTIM1->ICR = LPTIM_ICR_ARROKCF;
  LPTIM1->CR |= LPTIM_CR_SNGSTRT;                    /* cuenta una vez hasta ARR    */
}

bool power_lptim_vencio(void)
{
  if (!s_lptim_fin) return false;
  s_lptim_fin = false;
  return true;
}

uint32_t power_mpu_avisos(void)
{
  __disable_irq();
  uint32_t n = s_mpu_avisos;
  s_mpu_avisos = 0U;
  __enable_irq();
  return n;
}

void power_dormir(void)
{
  uart_vaciar();
  HAL_SuspendTick();                  /* que el SysTick no lo despierte cada 1 ms */

  /* Con las interrupciones bloqueadas, una interrupcion pendiente igual despierta
   * al WFI: asi no se pierde un aviso que llega justo antes de dormir. */
  __disable_irq();
  if (!hay_pendiente())
  {
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
  }
  __enable_irq();                     /* aca se atiende la interrupcion que lo desperto */

  HAL_ResumeTick();
  /* Nota: HAL_GetTick() no avanza durante el STOP2. La grilla usa la hora del RTC;
   * HAL_GetTick() solo se usa para esperas cortas estando despierto. */
}

void power_esperar(void)
{
  __WFI();
}

void power_esperar_desde(uint32_t t0, uint32_t ms)
{
  while ((HAL_GetTick() - t0) < ms)
  {
    __WFI();
  }
}

/* Reemplazo de la HAL_Delay() de la HAL ("weak"): misma logica (suma 1 tick
 * para garantizar la espera minima), pero dormido en vez de dando vueltas. */
void HAL_Delay(uint32_t Delay)
{
  uint32_t t0   = HAL_GetTick();
  uint32_t wait = Delay;

  if (wait < HAL_MAX_DELAY) wait += (uint32_t)uwTickFreq;
  power_esperar_desde(t0, wait);
}
