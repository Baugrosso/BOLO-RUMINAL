/**
 * @file    app_bolo.c
 * @brief   Aplicacion del nodo BOLO_TX. Coordina los modulos; no conoce registros,
 *          no arma textos y no sabe como se duerme: cada tarea la delega.
 */
#include "app_bolo.h"
#include "bolo_config.h"
#include "bolo_types.h"
#include "sensor_acc.h"
#include "sensor_temp.h"
#include "motion.h"
#include "thermal.h"
#include "report.h"
#include "radio.h"
#include "bolo_packet.h"
#include "bolo_power.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>      /* snprintf: arma el texto del PING */

/* ===================== Variables privadas ===================== */
static I2C_HandleTypeDef  *s_hi2c    = NULL;
static SPI_HandleTypeDef  *s_hspi    = NULL;   /* para reiniciar la radio si no despierta */
static IWDG_HandleTypeDef *s_hiwdg   = NULL;
static uint32_t           s_ciclo   = 0;
static bool               s_acc_ok  = false;
static bool               s_temp_ok = false;
static bool               s_radio_ok = false;   /* la radio quedo lista al arrancar */
static uint8_t            s_reinicio = PKT_RST_DESCONOCIDA; /* viaja en cada paquete */

/* El paquete usa los mismos numeros que los estados internos: si alguien los
 * cambia, esto deja de compilar en vez de mandar datos equivocados. */
_Static_assert(BOLO_ESTADO_MOV_FUERTE == PKT_MOV_FUERTE && BOLO_ESTADO_DESCONOCIDO == PKT_MOV_DESCONOCIDO,
               "bolo_estado_t y PKT_MOV_* tienen que coincidir");
_Static_assert(TERM_SUBA == PKT_TERM_SUBA && TERM_SIN_DATO == PKT_TERM_SIN_DATO,
               "term_estado_t y PKT_TERM_* tienen que coincidir");

/* ===================== Funciones privadas ===================== */
static void led(bool encendido)
{
#if BOLO_LED_SENSADO
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, encendido ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  (void)encendido;
#endif
}

#if BOLO_DIAG_I2C
static void escaneo_i2c(void)
{
  uint8_t dirs[8];
  uint8_t n = 0;

  for (uint8_t addr = 1; addr < 128U && n < sizeof(dirs); addr++)
  {
    if (HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(addr << 1), 2, 10) == HAL_OK)
    {
      dirs[n++] = addr;
    }
  }
  report_i2c(dirs, n);
}
#endif

/* Destraba el bus I2C y reinicia el periferico.
 * Si un sensor se quedo a mitad de un byte (por ejemplo, perdio y recupero la
 * alimentacion), puede quedar sosteniendo SDA en bajo y el bus queda "trabado".
 * Remedio estandar: hasta 9 pulsos de reloj en SCL para que termine de soltar
 * el byte, una condicion STOP, y despues el periferico arranca de cero. */
static void i2c_recuperar(void)
{
  GPIO_InitTypeDef g = {0};

  HAL_I2C_DeInit(s_hi2c);                       /* suelta los pines (MspDeInit) */

  HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SCL_PIN | BOLO_I2C_SDA_PIN, GPIO_PIN_SET);
  g.Pin   = BOLO_I2C_SCL_PIN | BOLO_I2C_SDA_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_OD;                /* drenador abierto, como el bus */
  g.Pull  = GPIO_NOPULL;                        /* los pull-up estan en los modulos */
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOLO_I2C_PORT, &g);

  for (uint8_t k = 0; k < 9U; k++)              /* ~10 us por medio ciclo: <100 kHz */
  {
    HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SCL_PIN, GPIO_PIN_RESET);
    for (volatile uint32_t d = 0; d < 50U; d++) { }
    HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SCL_PIN, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 50U; d++) { }
  }

  /* STOP: SDA sube mientras SCL esta en alto */
  HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SDA_PIN, GPIO_PIN_RESET);
  for (volatile uint32_t d = 0; d < 50U; d++) { }
  HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SDA_PIN, GPIO_PIN_SET);

  HAL_I2C_Init(s_hi2c);                         /* MspInit vuelve a poner los pines en I2C */
}

/* Lee por que arranco el micro y deja todo limpio para el proximo reinicio.
 * Orden importante: un HardFault y una carga desde CubeIDE usan los dos el reset
 * por software; lo que los distingue es la marca en el registro de backup. */
static const char *causa_reinicio(bool *anomalo)
{
  const char *c;
  *anomalo = true;

  if (RTC->BKP0R == BOLO_MARCA_HARDFAULT)        { c = "HARDFAULT";                       s_reinicio = PKT_RST_HARDFAULT; }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) { c = "WATCHDOG (el programa se trabo)"; s_reinicio = PKT_RST_WATCHDOG;  }
  else
  {
    *anomalo = false;
    if      (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) { c = "encendido (o caida de tension)";     s_reinicio = PKT_RST_ENCENDIDO; }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) { c = "por software (carga desde CubeIDE)"; s_reinicio = PKT_RST_SOFTWARE;  }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) { c = "boton de reset";                     s_reinicio = PKT_RST_BOTON;     }
    else                                          { c = "desconocida";                        s_reinicio = PKT_RST_DESCONOCIDA; }
  }

  RTC->BKP0R = 0U;                 /* se borra la marca (acceso habilitado en SystemClock_Config) */
  __HAL_RCC_CLEAR_RESET_FLAGS();   /* las banderas se acumulan si no se borran */
  return c;
}

/* g -> decimas de mg, redondeado y limitado a lo que entra en 16 bits */
static uint16_t a_dmg(float g)
{
  float v = g * 10000.0f + 0.5f;
  if (v < 0.0f)     v = 0.0f;
  if (v > 65535.0f) v = 65535.0f;
  return (uint16_t)v;
}

/* Pasa los datos del ciclo a las unidades del paquete (ver bolo_packet.h) */
static void armar_paquete(const bolo_data_t *d, bolo_pkt_t *p)
{
  *p = (bolo_pkt_t){ 0 };

  p->nodo            = BOLO_NODO_ID;
  p->seq             = (uint16_t)d->ciclo;
  p->mov             = (uint8_t)d->estado;
  p->term            = (uint8_t)d->term_estado;
  p->temp_ok         = d->temp_ok;
  p->ventana_cortada = d->ventana_cortada;
  p->bus_recuperado  = d->bus_recuperado;
  p->reinicio        = s_reinicio;

  p->desvio_ejes_dmg = a_dmg(d->desvio_max);
  p->desvio_mag_dmg  = a_dmg(d->desvio_mag);

  if (d->temp_ok)
  {
    uint32_t h = ((uint32_t)d->hum_centi + 50U) / 100U;   /* centesimas de % -> % */
    p->temp_centi  = d->temp_centi;
    p->delta_centi = d->temp_delta_centi;
    p->hum_pct     = (uint8_t)((h > 100U) ? 100U : h);
  }

  p->evento_fin = (d->term_fin_tipo == TERM_BAJADA || d->term_fin_tipo == TERM_SUBA);
  if (p->evento_fin)
  {
    p->evento_suba      = (d->term_fin_tipo == TERM_SUBA);
    p->ev_ciclos        = (uint8_t)((d->term_fin_ciclos > 255U) ? 255U : d->term_fin_ciclos);
    p->ev_extremo_centi = d->term_fin_extremo_centi;
  }
}

/* Dispara una medicion de temperatura. Si el sensor no responde, queda marcado
 * para reiniciarse y se avisa que hay que destrabar el bus. */
static bool temp_disparar(bool *falla_bus)
{
  if (temp_start()) return true;
  s_temp_ok  = false;
  *falla_bus = true;
  return false;
}

/* Encendido completo de la radio (TCXO, calibracion y configuracion LoRa) e informe */
static bool radio_probar(SPI_HandleTypeDef *hspi)
{
  radio_diag_t diag;
  bool ok = radio_init(hspi, &diag);

  report_radio(&diag);
  return ok;
}

/* ===================== Funciones publicas ===================== */
void app_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart, RTC_HandleTypeDef *hrtc,
              IWDG_HandleTypeDef *hiwdg, SPI_HandleTypeDef *hspi)
{
  bool anomalo;

  s_hi2c  = hi2c;
  s_hiwdg = hiwdg;   /* el IWDG ya esta corriendo: lo arranco MX_IWDG_Init() */
  s_hspi  = hspi;
  power_init(hrtc, huart);   /* reposo en STOP2 con el RTC como despertador */

#ifdef DEBUG
  /* Con el debugger en pausa el watchdog tambien se pausa (si no, resetea a los 25 s) */
  __HAL_DBGMCU_FREEZE_IWDG();
#endif

  thermal_reset();     /* la base termica se aprende desde la primera medicion */

  report_init(huart);
  report_banner();
  const char *causa = causa_reinicio(&anomalo);
  report_reinicio(causa, anomalo);

#if BOLO_DIAG_I2C
  escaneo_i2c();
#endif

  s_acc_ok = acc_init(hi2c);
  report_init_sensor(ACC_NOMBRE, s_acc_ok, acc_id());

  s_temp_ok = temp_init(hi2c);
  report_init_sensor(TEMP_NOMBRE, s_temp_ok, s_temp_ok ? TEMP_I2C_ADDR : 0U);

  s_radio_ok = radio_probar(hspi);   /* si falla, el bolo sigue midiendo igual */
  if (s_radio_ok) (void)radio_dormir();   /* duerme hasta la primera transmision */
}


void app_run(void)
{
  bolo_data_t datos = {0};      /* estado arranca en DESCONOCIDO */
  acc_raw_t   m;
  bool        falla_bus = false;  /* algo fallo en el I2C este ciclo -> destrabar al final */

  /* Unico lugar donde se refresca el watchdog: si el ciclo se traba en cualquier
   * punto, no se vuelve a llegar aca y el IWDG resetea el micro. */
  HAL_IWDG_Refresh(s_hiwdg);

  /* Encabezado del ciclo primero: los avisos de reintento quedan adentro del ciclo */
  report_sensando(s_ciclo);

#if (BOLO_PRUEBA_FALLA != 0)
  if (s_ciclo == 3U)
  {
    report_prueba_falla(BOLO_PRUEBA_FALLA);
    led(true);
  #if (BOLO_PRUEBA_FALLA == 1)
    while (1) { }                 /* cuelgue: nadie refresca el watchdog */
  #else
    __builtin_trap();             /* instruccion invalida -> HardFault */
  #endif
  }
#endif

  /* Si el acelerometro no inicializo (o fallo el ciclo anterior), se reintenta */
  if (!s_acc_ok)
  {
    s_acc_ok = acc_init(s_hi2c);
    report_init_sensor(ACC_NOMBRE, s_acc_ok, acc_id());
    if (!s_acc_ok) falla_bus = true;
  }

  /* Igual con el sensor de temperatura */
  if (!s_temp_ok)
  {
    s_temp_ok = temp_init(s_hi2c);
    report_init_sensor(TEMP_NOMBRE, s_temp_ok, s_temp_ok ? TEMP_I2C_ADDR : 0U);
    if (!s_temp_ok) falla_bus = true;
  }

  /* ---------- SENSAR ---------- */
  led(true);
  acc_wake();
  motion_reset(ACC_CUENTAS_POR_G);

  bool     temp_midiendo = false;
  uint32_t t0       = HAL_GetTick();
  uint32_t t        = t0;
  uint8_t  seguidas = 0;                                /* fallas consecutivas */

  for (uint32_t i = 0; i < ACC_N_MUESTRAS; i++)
  {
    power_esperar_desde(t, ACC_PERIODO_MS);            /* SLEEP hasta la proxima muestra */
    t += ACC_PERIODO_MS;                                /* turno contado desde el inicio */

    if (acc_read(&m))
    {
      motion_add(&m);
      seguidas = 0;
    }
    else
    {
      datos.error_sensor = true;
      datos.fallas_lectura++;
      if (++seguidas >= ACC_FALLAS_MAX) { datos.ventana_cortada = true; break; }
    }

    /* Cerca del final de la ventana se dispara la temperatura: el SHT31 mide solo
     * (~12,5 ms) mientras el acelerometro toma la ultima muestra */
    if (i == TEMP_INICIO_MUESTRA && s_temp_ok) temp_midiendo = temp_disparar(&falla_bus);

    /* Tope de tiempo: pase lo que pase, la ventana no se estira mas de 6 s */
    if ((HAL_GetTick() - t0) > VENTANA_MAX_MS) { datos.ventana_cortada = true; break; }
  }
  datos.ventana_ms = (uint16_t)(HAL_GetTick() - t0);

  /* Si la ventana se corto antes de disparar la temperatura, se mide igual:
   * que falle el acelerometro no tiene por que dejarnos sin temperatura */
  if (s_temp_ok && !temp_midiendo)
  {
    temp_midiendo = temp_disparar(&falla_bus);
    if (temp_midiendo) HAL_Delay(TEMP_T_MEDICION_MS);
  }

  acc_sleep();
  led(false);

  if (datos.fallas_lectura > 0U)
  {
    s_acc_ok  = false;          /* pudo perder la alimentacion: se reconfigura entero */
    falla_bus = true;
  }

  /* La medicion de temperatura ya termino: se lee el resultado */
  if (temp_midiendo)
  {
    datos.temp_ok = temp_read(&datos.temp_centi, &datos.hum_centi);
    if (!datos.temp_ok) { s_temp_ok = false; falla_bus = true; }
  }

  /* Si hubo fallas, el bus se destraba ahora; los sensores que fallaron se
   * reinician al principio del proximo ciclo (sin resetear la placa). */
  if (falla_bus)
  {
    i2c_recuperar();
    datos.bus_recuperado = true;
  }

  /* ---------- PROCESAR ---------- */
  datos.ciclo = s_ciclo++;
  motion_compute(&datos);
  thermal_update(&datos);
  if (datos.ventana_cortada) datos.estado = BOLO_ESTADO_DESCONOCIDO;  /* dato incompleto */

  /* ---------- REPORTAR ---------- */
  report_resultado(&datos);

  /* ---------- TRANSMITIR (etapa 3: los datos del ciclo, 18 bytes) ---------- */
  if (s_radio_ok)
  {
    bolo_pkt_t pkt;
    uint8_t    buf[PKT_LARGO_V1];
    char       msg[24];
    radio_tx_t tx;

    armar_paquete(&datos, &pkt);
    uint8_t n = bolo_pkt_encode(&pkt, buf);

    /* La radio paso el reposo dormida. Si no despierta bien, se rehace la
     * configuracion completa (radio_probar informa el resultado como al arrancar). */
    if (!radio_despertar())
    {
      s_radio_ok = radio_probar(s_hspi);
    }

    if (s_radio_ok)
    {
      (void)snprintf(msg, sizeof(msg), "datos #%u, %u B", (unsigned)pkt.seq, (unsigned)n);
      bool enviado = radio_send(buf, n, &tx);
      report_radio_tx(msg, &tx);

      /* Si fallo despues de dormir, se reconfigura la radio y se intenta una vez mas */
      if (!enviado)
      {
        s_radio_ok = radio_probar(s_hspi);
        if (s_radio_ok)
        {
          (void)radio_send(buf, n, &tx);
          report_radio_tx(msg, &tx);
        }
      }

      if (s_radio_ok) (void)radio_dormir();   /* hasta la proxima transmision: < 1 uA */
    }
  }

  /* ---------- REPOSO ---------- */
  report_durmiendo(T_REPOSO_S);
  power_dormir_s(T_REPOSO_S);                           /* STOP2, lo despierta el RTC */
}
