/**
 * @file    bolo_power.c
 * @brief   Bajo consumo del BOLO_TX: reposo en STOP2 despertado por el RTC.
 *
 * Dos niveles de sueño:
 *  - SLEEP (esperas cortas, durante la ventana): solo se para el nucleo; los
 *    perifericos y el SysTick siguen andando y cualquier interrupcion lo despierta.
 *  - STOP2 (el reposo largo): ver abajo.
 *
 * Que es STOP2: se apagan los relojes rapidos (MSI) y casi todo el chip, pero se
 * conservan la RAM y los registros de los perifericos. Siguen vivos el LSE + RTC
 * (el despertador) y el LSI + IWDG (el watchdog sigue contando: el ciclo entero
 * tiene que entrar en sus ~25 s). Al despertar, el micro sigue en la linea
 * siguiente al WFI, con el MSI a 4 MHz como antes: no hay que reconfigurar nada.
 *
 * El RTC cuenta en segundos (CK_SPRE = 32768 Hz / 128 / 256 = 1 Hz) con el
 * contador de despertar de 16 bits: de 1 s a ~18 h.
 */
#include "bolo_power.h"
#include "bolo_config.h"
#include <stdbool.h>
#include <stdio.h>

#define UART_VACIADO_MS     20U       /* tope para esperar que salga el ultimo byte */
#define RTC_WUT_MAX_S       65536U    /* contador de 16 bits a 1 Hz                 */

static RTC_HandleTypeDef  *s_hrtc     = NULL;
static UART_HandleTypeDef *s_huart    = NULL;
static volatile bool       s_desperto = false;   /* lo pone la interrupcion del RTC */

/* La llama HAL_RTCEx_WakeUpTimerIRQHandler() (desde RTC_WKUP_IRQHandler). */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  (void)hrtc;
  s_desperto = true;
}

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

/* Espera en SLEEP: __WFI() apaga el nucleo hasta la proxima interrupcion (el
 * SysTick, cada 1 ms). Despierta, mira la hora y si falta vuelve a dormir. */
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

void power_init(RTC_HandleTypeDef *hrtc, UART_HandleTypeDef *huart)
{
  s_hrtc  = hrtc;
  s_huart = huart;

#if BOLO_DBG_EN_STOP
  HAL_DBGMCU_EnableDBGStopMode();     /* el debugger sobrevive al STOP2 (consume mas) */
#else
  HAL_DBGMCU_DisableDBGStopMode();    /* consumo real                                  */
#endif

  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);  /* despierta con el MSI */
  (void)HAL_RTCEx_DeactivateWakeUpTimer(s_hrtc);
}

void power_dormir_s(uint32_t segundos)
{
  if (segundos == 0U) return;
  if (segundos > RTC_WUT_MAX_S) segundos = RTC_WUT_MAX_S;

  uart_vaciar();

  (void)HAL_RTCEx_DeactivateWakeUpTimer(s_hrtc);
  s_desperto = false;

  /* El RTC interrumpe cuando el contador llega a 0: N-1 equivale a N segundos */
  if (s_hrtc == NULL ||
      HAL_RTCEx_SetWakeUpTimer_IT(s_hrtc, segundos - 1U, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    HAL_Delay(segundos * 1000U);      /* sin despertador: espera despierto (no se cuelga) */
    return;
  }

  HAL_SuspendTick();                  /* que el SysTick no lo despierte cada 1 ms */

  /* Con las interrupciones bloqueadas, una interrupcion pendiente igual despierta
   * al WFI: asi no se pierde el aviso del RTC si llega justo antes de dormir.
   * Si lo despierta otra cosa, vuelve a dormir hasta que avise el RTC. */
  for (;;)
  {
    __disable_irq();
    if (s_desperto) { __enable_irq(); break; }
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    __enable_irq();                   /* aca se atiende la interrupcion que lo desperto */
  }

  HAL_ResumeTick();
  (void)HAL_RTCEx_DeactivateWakeUpTimer(s_hrtc);

  /* El SysTick estuvo parado: se le suma lo dormido para que HAL_GetTick() siga
   * siendo el tiempo real desde el arranque. */
  uwTick += segundos * 1000U;
}
