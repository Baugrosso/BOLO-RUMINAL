/**
 * @file    bolo_reloj.c
 * @brief   Hora UNIX sobre el RTC, tick de 1 s y estado persistente en backup.
 *
 *  El contador s_unix lo incrementa la interrupcion del wake-up timer (1 Hz, del
 *  LSE): leerlo no requiere sincronizar el calendario despues de cada Stop 2. El
 *  calendario del RTC se lee solo al arrancar (sobrevive al reset) y se escribe
 *  cuando llega la hora de la gateway.
 */
#include "bolo_reloj.h"
#include "bolo_config.h"

#ifndef BOLO_HOST
#include "main.h"
#endif

/* ===================== Conversion de fechas ===================== */
/* Dias desde 1970-01-01 (algoritmo "days from civil", calendario gregoriano) */
static int32_t dias_desde_civil(int32_t y, uint32_t m, uint32_t d)
{
  y -= (m <= 2U) ? 1 : 0;
  int32_t  era = y / 400;
  uint32_t yoe = (uint32_t)(y - era * 400);
  uint32_t doy = (153U * (m + ((m > 2U) ? (uint32_t)-3 : 9U)) + 2U) / 5U + d - 1U;
  uint32_t doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  return era * 146097 + (int32_t)doe - 719468;
}

uint32_t reloj_a_unix(uint16_t anio, uint8_t mes, uint8_t dia,
                      uint8_t hora, uint8_t min, uint8_t seg)
{
  int32_t dias = dias_desde_civil((int32_t)anio, mes, dia);
  return (uint32_t)dias * 86400U + (uint32_t)hora * 3600U + (uint32_t)min * 60U + seg;
}

void reloj_de_unix(uint32_t t_unix, uint16_t *anio, uint8_t *mes, uint8_t *dia,
                   uint8_t *hora, uint8_t *min, uint8_t *seg)
{
  uint32_t s = t_unix % 86400U;
  int32_t  z = (int32_t)(t_unix / 86400U) + 719468;
  int32_t  era = z / 146097;
  uint32_t doe = (uint32_t)(z - era * 146097);
  uint32_t yoe = (doe - doe / 1460U + doe / 36524U - doe / 146096U) / 365U;
  int32_t  y   = (int32_t)yoe + era * 400;
  uint32_t doy = doe - (365U * yoe + yoe / 4U - yoe / 100U);
  uint32_t mp  = (5U * doy + 2U) / 153U;
  uint32_t d   = doy - (153U * mp + 2U) / 5U + 1U;
  uint32_t m   = (mp < 10U) ? mp + 3U : mp - 9U;

  *anio = (uint16_t)(y + ((m <= 2U) ? 1 : 0));
  *mes  = (uint8_t)m;
  *dia  = (uint8_t)d;
  *hora = (uint8_t)(s / 3600U);
  *min  = (uint8_t)((s / 60U) % 60U);
  *seg  = (uint8_t)(s % 60U);
}

/* ===================== Hora ===================== */
static volatile uint32_t s_unix = 946684800U;   /* 2000-01-01, lo que tiene el RTC al encender */
static volatile bool     s_tick = false;

uint32_t reloj_ahora(void)
{
  return s_unix;                                /* lectura de 32 bits: atomica en el M4 */
}

bool reloj_tick_pendiente(void)
{
  return s_tick;
}

bool reloj_tick(void)
{
  if (!s_tick) return false;
  s_tick = false;
  return true;
}

#ifdef BOLO_HOST
/* En la PC la hora la maneja el test */
int32_t reloj_fijar(uint32_t t_unix)
{
  int32_t delta = (int32_t)(t_unix - s_unix);
  s_unix = t_unix;
  return delta;
}
#else
static RTC_HandleTypeDef *s_hrtc = NULL;

/* Lo llama HAL_RTCEx_WakeUpTimerIRQHandler() (desde RTC_WKUP_IRQHandler), cada 1 s */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  (void)hrtc;
  s_unix++;
  s_tick = true;
}

void reloj_init(RTC_HandleTypeDef *hrtc)
{
  RTC_TimeTypeDef t = {0};
  RTC_DateTypeDef d = {0};

  s_hrtc = hrtc;

  /* Leer hora y fecha (en ese orden: leer la hora congela la fecha hasta leerla) */
  if (HAL_RTC_GetTime(hrtc, &t, RTC_FORMAT_BIN) == HAL_OK &&
      HAL_RTC_GetDate(hrtc, &d, RTC_FORMAT_BIN) == HAL_OK &&
      d.Month >= 1U && d.Month <= 12U && d.Date >= 1U && d.Date <= 31U)
  {
    s_unix = reloj_a_unix((uint16_t)(2000U + d.Year), d.Month, d.Date,
                          t.Hours, t.Minutes, t.Seconds);
  }

  /* Tick periodico de 1 s: CK_SPRE = 32768 / 128 / 256 = 1 Hz; recarga 0 -> cada 1 s */
  (void)HAL_RTCEx_DeactivateWakeUpTimer(hrtc);
  (void)HAL_RTCEx_SetWakeUpTimer_IT(hrtc, 0U, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
}

int32_t reloj_fijar(uint32_t t_unix)
{
  RTC_TimeTypeDef t = {0};
  RTC_DateTypeDef d = {0};
  uint16_t anio;
  uint8_t  mes, dia, hh, mm, ss;

  if (t_unix < 946684800U) return 0;              /* antes de 2000: el RTC no lo representa */

  reloj_de_unix(t_unix, &anio, &mes, &dia, &hh, &mm, &ss);
  if (anio > 2099U) return 0;

  t.Hours = hh; t.Minutes = mm; t.Seconds = ss;
  t.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  t.StoreOperation = RTC_STOREOPERATION_RESET;
  d.Year  = (uint8_t)(anio - 2000U);
  d.Month = mes;
  d.Date  = dia;
  /* Dia de la semana 1..7 (lunes = 1). 1970-01-01 fue jueves. */
  d.WeekDay = (uint8_t)(((t_unix / 86400U + 3U) % 7U) + 1U);

  if (s_hrtc != NULL)
  {
    (void)HAL_RTC_SetTime(s_hrtc, &t, RTC_FORMAT_BIN);
    (void)HAL_RTC_SetDate(s_hrtc, &d, RTC_FORMAT_BIN);
  }

  __disable_irq();
  int32_t delta = (int32_t)(t_unix - s_unix);
  s_unix = t_unix;
  __enable_irq();
  return delta;
}
#endif

/* ===================== Estado persistente ===================== */
/* BKP1R: [31:24] marca 0xB0 | [23:16] banderas | [15:8] ranura | [7:0] potencia (int8)
 * BKP2R: [31:16] n. de periodo | [15:8] contacto | [7:0] contactos sin ACK
 * (BKP0R es la marca de HardFault) */
#define PERSIST_MARCA       0xB0U
#define PFLAG_HORA          0x01U
#define PFLAG_RANURA        0x02U

#ifdef BOLO_HOST
static uint32_t s_bkp1 = 0U, s_bkp2 = 0U;
#define BKP1  s_bkp1
#define BKP2  s_bkp2
#else
#define BKP1  (RTC->BKP1R)
#define BKP2  (RTC->BKP2R)
#endif

void persist_leer(bolo_persist_t *p)
{
  uint32_t r1 = BKP1;
  uint32_t r2 = BKP2;

  if ((r1 >> 24) != PERSIST_MARCA)
  {
    /* Nada guardado (encendido): valores por defecto */
    *p = (bolo_persist_t){ 0 };
    p->potencia_dbm = (int8_t)RADIO_POTENCIA_DBM;
    return;
  }

  uint8_t flags = (uint8_t)(r1 >> 16);
  p->hora_sincronizada = (flags & PFLAG_HORA) != 0U;
  p->ranura_valida     = (flags & PFLAG_RANURA) != 0U;
  p->ranura            = (uint8_t)(r1 >> 8);
  p->potencia_dbm      = (int8_t)(uint8_t)r1;
  p->n_periodo         = (uint16_t)(r2 >> 16);
  p->contacto          = (uint8_t)(r2 >> 8);
  p->sin_ack           = (uint8_t)r2;

  if (p->ranura >= RANURAS_POR_PERIODO) p->ranura_valida = false;
}

void persist_guardar(const bolo_persist_t *p)
{
  uint8_t flags = (uint8_t)((p->hora_sincronizada ? PFLAG_HORA : 0U) |
                            (p->ranura_valida     ? PFLAG_RANURA : 0U));

  BKP1 = ((uint32_t)PERSIST_MARCA << 24) | ((uint32_t)flags << 16) |
         ((uint32_t)p->ranura << 8) | (uint32_t)(uint8_t)p->potencia_dbm;
  BKP2 = ((uint32_t)p->n_periodo << 16) | ((uint32_t)p->contacto << 8) | p->sin_ack;
}
