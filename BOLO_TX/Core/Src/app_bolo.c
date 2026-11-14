/**
 * @file    app_bolo.c
 * @brief   Aplicacion del BOLO_TX, por eventos. Coordina los modulos; no conoce
 *          registros ni arma textos.
 *
 *  El micro duerme en STOP2 y se despierta por:
 *   - el tick de 1 s del RTC  -> grilla: cierre de epoca (20 s), medicion de
 *                                temperatura (60 s), media de 10 min, periodo (6 h)
 *                                y hora de contacto; refresco del watchdog
 *   - el INT del MPU-6050     -> una muestra de aceleracion (5 Hz)
 *   - el LPTIM1               -> fin de la medicion del SHT31 (16 ms)
 *
 *  Contactos (T12): en la ranura propia (2 min x ranura despues de cada limite de
 *  6 h) + espera al azar de 0-20 s; hasta 3 intentos cada 4 min. Sin ranura:
 *  primer contacto a 1-5 min del arranque y reintentos cada 1-3 min.
 */
#include "app_bolo.h"
#include "bolo_config.h"
#include "bolo_types.h"
#include "bolo_reloj.h"
#include "bolo_power.h"
#include "bolo_util.h"
#include "bolo_packet.h"
#include "sensor_acc.h"
#include "sensor_temp.h"
#include "motion.h"
#include "periodo.h"
#include "almacen.h"
#include "contacto.h"
#include "radio.h"
#include "report.h"
#include <stdbool.h>
#include <stddef.h>

/* Una hora anterior a 2024-01-01 solo puede ser la del RTC recien encendido */
#define BOLO_UNIX_MIN_VALIDA   1704067200U

/* ===================== Variables privadas ===================== */
static I2C_HandleTypeDef  *s_hi2c  = NULL;
static SPI_HandleTypeDef  *s_hspi  = NULL;
static IWDG_HandleTypeDef *s_hiwdg = NULL;

static bolo_persist_t s_p;                 /* espejo de los registros de backup */
static bool     s_acc_ok   = false;
static bool     s_temp_ok  = false;
static bool     s_radio_ok = false;
static uint8_t  s_causa    = RST_DESCONOCIDO;  /* va en el primer periodo despues del arranque */

/* Grilla */
static uint32_t s_seg       = 0U;          /* ultimo segundo procesado               */
static uint32_t s_epoca_id  = 0U;          /* epoca en curso = t / 20                */
static uint32_t s_th_idx    = 0U;          /* media de 10 min en curso (desde la marca) */
static uint32_t s_periodo_id = 0U;         /* periodo en curso = t / T_PERIODO_S     */
static uint32_t s_minuto_id = 0U;          /* ultimo minuto con medicion de temperatura */
static bool     s_sht_midiendo = false;
static uint32_t s_sht_t     = 0U;

/* Contactos */
static bool     s_en_contacto = false;
static uint32_t s_t_contacto  = 0U;        /* hora del proximo contacto              */
static uint32_t s_ciclo       = 0U;        /* bloque de 6 h del proximo contacto     */
static uint8_t  s_intento     = 0U;        /* 0..2 dentro del ciclo                  */

/* ===================== Auxiliares ===================== */
static void led(bool encendido)
{
#if BOLO_LED_CONTACTO
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
    if (HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(addr << 1), 2, 10) == HAL_OK) dirs[n++] = addr;
  }
  report_i2c(dirs, n);
}
#endif

/* Destraba el bus I2C y reinicia el periferico: hasta 9 pulsos en SCL para que un
 * sensor a mitad de byte suelte SDA, una condicion STOP, y el periferico de cero. */
static void i2c_recuperar(void)
{
  GPIO_InitTypeDef g = {0};

  HAL_I2C_DeInit(s_hi2c);

  HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SCL_PIN | BOLO_I2C_SDA_PIN, GPIO_PIN_SET);
  g.Pin   = BOLO_I2C_SCL_PIN | BOLO_I2C_SDA_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOLO_I2C_PORT, &g);

  for (uint8_t k = 0; k < 9U; k++)
  {
    HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SCL_PIN, GPIO_PIN_RESET);
    for (volatile uint32_t d = 0; d < 50U; d++) { }
    HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SCL_PIN, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 50U; d++) { }
  }
  HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SDA_PIN, GPIO_PIN_RESET);
  for (volatile uint32_t d = 0; d < 50U; d++) { }
  HAL_GPIO_WritePin(BOLO_I2C_PORT, BOLO_I2C_SDA_PIN, GPIO_PIN_SET);

  HAL_I2C_Init(s_hi2c);
}

/* Por que arranco el micro. Un HardFault y una carga desde CubeIDE usan los dos el
 * reset por software; los distingue la marca en el registro de backup. */
static const char *causa_reinicio(bool *anomalo)
{
  const char *c;
  *anomalo = true;

  if (RTC->BKP0R == BOLO_MARCA_HARDFAULT)        { c = "HARDFAULT";                          s_causa = RST_HARDFAULT; }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) { c = "WATCHDOG (el programa se trabo)";    s_causa = RST_WATCHDOG;  }
  else
  {
    *anomalo = false;
    if      (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) { c = "encendido (o caida de tension)";     s_causa = RST_ENCENDIDO; }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) { c = "por software (carga desde CubeIDE)"; s_causa = RST_SOFTWARE;  }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) { c = "boton de reset";                     s_causa = RST_BOTON;     }
    else                                          { c = "desconocida";                        s_causa = RST_DESCONOCIDO; }
  }

  RTC->BKP0R = 0U;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  return c;
}

static bool radio_probar(void)
{
  radio_diag_t diag;
  bool ok = radio_init(s_hspi, s_p.potencia_dbm, &diag);
  report_radio(&diag);
  if (ok) (void)radio_dormir();
  return ok;
}

/* ===================== Periodos ===================== */
static void abrir_periodo(uint32_t marca)
{
  periodo_abrir(marca, !s_p.hora_sincronizada, s_causa);
  s_causa      = RST_NINGUNO;
  s_periodo_id = marca / T_PERIODO_S;
  s_epoca_id   = marca / T_EPOCA_S;
  s_th_idx     = 0U;
  motion_reset();
}

/* Cierra el periodo en armado y lo guarda.
 * completo: termino la grilla de 6 h -> se completan con "sin dato" las epocas y
 *           medias que falten hasta el final (la grilla queda exacta).
 * parcial : (arranque, salto de hora) se guarda lo que haya. */
static void cerrar_periodo(bool completo)
{
  if (!periodo_abierto()) return;

  if (completo)
  {
    uint32_t fin   = (s_periodo_id + 1U) * T_PERIODO_S;
    uint32_t marca = periodo_marca();
    uint32_t esp_ep = (fin > marca) ? (fin - marca) / T_EPOCA_S : 0U;
    uint32_t esp_th = (fin > marca) ? (fin - marca + T_MEDIA_TH_S - 1U) / T_MEDIA_TH_S : 0U;

    while (periodo_n_th() < esp_th && periodo_cerrar_th()) { }
    while (periodo_n_epocas() < esp_ep && periodo_epoca(NULL)) { }
  }
  else
  {
    if (periodo_th_pendiente()) (void)periodo_cerrar_th();
    if (periodo_n_epocas() == 0U) { periodo_descartar(); return; }   /* < 20 s de datos */
  }

  uint16_t mv = vdd_medir_mv();
  periodo_cierre_t c = { s_p.n_periodo, vdd_codigo(mv), s_p.sin_ack };
  periodo_info_t   info;

  periodo_cerrar(&c, &info);
  s_p.n_periodo++;
  persist_guardar(&s_p);
  report_periodo(&info, mv, !completo);
}

/* Cierra la epoca en curso (aunque no hayan pasado los 20 s) */
static void cerrar_epoca(void)
{
  epoca_t ep;

  motion_epoca(&ep);
  if (s_acc_ok && ep.n_muestras == 0U) s_acc_ok = false;   /* no llegan avisos: se reconfigura */
  (void)periodo_epoca(&ep);
  report_epoca(s_epoca_id * T_EPOCA_S, &ep);
  motion_reset();

  /* El acelerometro que fallo se reintenta una vez por epoca */
  if (!s_acc_ok)
  {
    periodo_falla_acc();
    s_acc_ok = acc_init(s_hi2c);
    if (!s_acc_ok) i2c_recuperar();
  }
}

/* ===================== Sensores ===================== */
static void iniciar_sht(uint32_t t)
{
  if (!s_temp_ok)
  {
    s_temp_ok = temp_init(s_hi2c);
    if (!s_temp_ok)
    {
      periodo_falla_temp();
      i2c_recuperar();
      report_th(t, false, 0, 0U);
      return;
    }
  }

  if (temp_start())
  {
    s_sht_midiendo = true;
    s_sht_t        = t;
    power_lptim_iniciar(TEMP_T_MEDICION_MS);   /* 16 ms en STOP2 */
  }
  else
  {
    s_temp_ok = false;
    periodo_falla_temp();
    i2c_recuperar();
  }
}

static void leer_sht(void)
{
  int16_t  t_c;
  uint16_t h_c;

  if (!s_sht_midiendo) return;
  s_sht_midiendo = false;

  if (temp_read(&t_c, &h_c))
  {
    periodo_muestra_th(t_c, h_c);
    report_th(s_sht_t, true, t_c, h_c);
  }
  else
  {
    s_temp_ok = false;
    periodo_falla_temp();
    i2c_recuperar();
    report_th(s_sht_t, false, 0, 0U);
  }
}

static void leer_mpu(void)
{
  acc_raw_t m;

  if (!s_acc_ok) return;
  if (acc_read(&m))                          /* leer tambien borra el aviso del INT */
  {
    motion_add(&m);
  }
  else
  {
    s_acc_ok = false;
    i2c_recuperar();
  }
}

/* ===================== Grilla ===================== */
static void procesar_tiempo(uint32_t t)
{
  if (t == s_seg) return;
  s_seg = t;

  if (!periodo_abierto()) abrir_periodo((t / T_EPOCA_S) * T_EPOCA_S);

  /* ---- Epoca (20 s) y periodo (6 h) ---- */
  uint32_t e = t / T_EPOCA_S;
  if (e > s_epoca_id)
  {
    cerrar_epoca();

    uint32_t p = t / T_PERIODO_S;
    if (p > s_periodo_id)
    {
      cerrar_periodo(true);
      abrir_periodo(p * T_PERIODO_S);
    }
    else
    {
      /* Epocas salteadas (el micro estuvo ocupado o la hora salto un poco) */
      for (uint32_t k = s_epoca_id + 1U; k < e; k++) (void)periodo_epoca(NULL);
    }
    s_epoca_id = e;
  }

  /* ---- Media de temperatura y humedad (10 min) ---- */
  uint32_t marca = periodo_marca();
  if (t >= marca)
  {
    uint32_t j = (t - marca) / T_MEDIA_TH_S;
    if (j > s_th_idx)
    {
      for (uint32_t k = s_th_idx; k < j; k++) (void)periodo_cerrar_th();
      s_th_idx = j;
      periodo_respaldar();                   /* foto del periodo cada 10 min */
    }
  }

  /* ---- Temperatura (cada 60 s) ---- */
  uint32_t mi = t / T_SHT_S;
  if (mi > s_minuto_id && !s_sht_midiendo)
  {
    s_minuto_id = mi;
    iniciar_sht(t);
  }
}

/* Se llama en cada despertar, y tambien durante los contactos */
static void servicio(void)
{
  HAL_IWDG_Refresh(s_hiwdg);
  (void)reloj_tick();
  procesar_tiempo(reloj_ahora());
  if (power_mpu_avisos() > 0U) leer_mpu();
  if (power_lptim_vencio())    leer_sht();
}

/* ===================== Hora de la gateway ===================== */
static void aplicar_hora(uint32_t gw)
{
  if (gw < 946684800U) return;                       /* 0 o invalida: no se toca */

  bool    primera = !s_p.hora_sincronizada;
  int32_t d       = (int32_t)(gw - reloj_ahora());

  if (primera || d > (int32_t)BOLO_SALTO_HORA_MAX_S || d < -(int32_t)BOLO_SALTO_HORA_MAX_S)
  {
    /* Salto grande (o primera hora): el periodo en curso se cierra como parcial y
     * se arranca otro en la grilla nueva. Los periodos cerrados sin hora se corrigen. */
    cerrar_epoca();
    cerrar_periodo(false);
    d = reloj_fijar(gw);
    if (primera) almacen_reestampar(d);
    s_p.hora_sincronizada = true;
    persist_guardar(&s_p);

    uint32_t t = reloj_ahora();
    abrir_periodo((t / T_EPOCA_S) * T_EPOCA_S);
    s_seg       = t;
    s_minuto_id = t / T_SHT_S;
  }
  else
  {
    /* Deriva chica: se absorbe (si adelanta, las epocas salteadas van como invalidas;
     * si atrasa, la epoca en curso dura un poco mas) */
    d = reloj_fijar(gw);
  }
  report_hora(gw, d, primera);
}

/* ===================== Contactos ===================== */
static void planificar(void)
{
  uint32_t ahora = reloj_ahora();

  if (!s_p.ranura_valida)
  {
    s_t_contacto = ahora + azar_rango(T_REINTENTO_MIN_S, T_REINTENTO_MAX_S);
    report_proximo(ahora, s_t_contacto);
    return;
  }

  /* Si el ciclo previsto quedo muy atras o muy adelante (arranque, salto de hora
   * en cualquier sentido), se reubica en la grilla actual */
  if ((uint64_t)s_ciclo * T_PERIODO_S + T_PERIODO_S < ahora)
  {
    s_ciclo   = ahora / T_PERIODO_S;
    s_intento = 0U;
  }
  else if (s_ciclo > ahora / T_PERIODO_S + 1U)
  {
    s_ciclo   = ahora / T_PERIODO_S + 1U;
    s_intento = 0U;
  }

  for (;;)
  {
    uint32_t base = s_ciclo * T_PERIODO_S +
                    ((uint32_t)s_p.ranura + CONT_SEPARACION_RANURAS * s_intento) * T_RANURA_S;
    if (base + T_ESPERA_HOLA_MAX_S > ahora)
    {
      uint32_t t = base + azar_rango(0U, T_ESPERA_HOLA_MAX_S);   /* espera al azar 0-20 s */
      s_t_contacto = (t > ahora) ? t : ahora + 1U;
      break;
    }
    /* Ese intento ya paso: el siguiente */
    if (++s_intento >= CONT_INTENTOS) { s_intento = 0U; s_ciclo++; }
  }
  report_proximo(ahora, s_t_contacto);
}

static void hacer_contacto(void)
{
  cont_res_t r;
  bool pide = !s_p.ranura_valida;

  s_en_contacto = true;
  s_p.contacto++;
  persist_guardar(&s_p);
  report_contacto_inicio(s_p.contacto, s_intento, pide);

  if (!s_radio_ok) s_radio_ok = radio_probar();

  if (s_radio_ok)
  {
    led(true);
    contacto_ejecutar(BOLO_NODO_ID, s_p.contacto, pide, servicio, &r);
    led(false);
    report_contacto_fin(&r);
    if (r.estado == CONT_FALLA_RADIO) s_radio_ok = radio_probar();   /* reconfigurar de cero */
  }
  else
  {
    r = (cont_res_t){ 0 };
    r.estado = CONT_FALLA_RADIO;
  }

  if (r.estado == CONT_OK)
  {
    bool ranura_nueva = false;

    /* Con cada ACK: hora de la gateway y comando */
    aplicar_hora(r.hora_gw);
    s_p.sin_ack = 0U;

    if (r.comando == PKT_CMD_RANURA && r.argumento < RANURAS_POR_PERIODO)
    {
      ranura_nueva      = !s_p.ranura_valida || s_p.ranura != r.argumento;
      s_p.ranura        = r.argumento;
      s_p.ranura_valida = true;
      if (ranura_nueva) report_ranura(s_p.ranura);
    }
    else if (r.comando == PKT_CMD_POTENCIA)
    {
      int8_t dbm = (int8_t)r.argumento;
      if (dbm >= -9 && dbm <= 22 && dbm != s_p.potencia_dbm)
      {
        s_p.potencia_dbm = dbm;
        if (radio_despertar() && radio_potencia(dbm)) report_potencia(dbm);
        (void)radio_dormir();
      }
    }

    /* El proximo contacto es en el ciclo siguiente. Con una ranura recien
     * asignada se cuenta desde ahora (los datos acaban de salir). */
    if (s_p.ranura_valida)
    {
      s_ciclo   = ranura_nueva ? (reloj_ahora() / T_PERIODO_S + 1U) : (s_ciclo + 1U);
      s_intento = 0U;
    }
  }
  else if (s_p.ranura_valida)
  {
    /* Hasta 3 intentos por ciclo; si fallan los 3, los datos quedan guardados */
    if (++s_intento >= CONT_INTENTOS)
    {
      s_intento = 0U;
      s_ciclo++;
      if (s_p.sin_ack < 0xFFU) s_p.sin_ack++;
    }
  }

  persist_guardar(&s_p);
  planificar();
  s_en_contacto = false;
}

/* ===================== Funciones publicas ===================== */
void app_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart, RTC_HandleTypeDef *hrtc,
              IWDG_HandleTypeDef *hiwdg, SPI_HandleTypeDef *hspi)
{
  bool anomalo;
  alm_estado_t est;

  s_hi2c  = hi2c;
  s_hiwdg = hiwdg;   /* el IWDG ya esta corriendo: lo arranco MX_IWDG_Init() */
  s_hspi  = hspi;

#ifdef DEBUG
  __HAL_DBGMCU_FREEZE_IWDG();   /* con el debugger en pausa el watchdog tambien se pausa */
#endif

  power_init(huart);
  reloj_init(hrtc);

  report_init(huart);
  report_banner();
  const char *causa = causa_reinicio(&anomalo);
  report_reinicio(causa, anomalo);

  persist_leer(&s_p);
  if (reloj_ahora() < BOLO_UNIX_MIN_VALIDA)
  {
    /* El RTC perdio la hora (volvio a 2000-01-01): la hora y la ranura ya no valen */
    s_p.hora_sincronizada = false;
    s_p.ranura_valida     = false;
  }

  /* El CRC valida lo que quedo en RAM: primero se prueba la unidad por hardware */
  report_crc(crc32_autotest());

  /* Lo que quedo en RAM de antes del reset */
  almacen_init();
  bool recuperado = periodo_recuperar();
  if (recuperado)
  {
    uint8_t causa_nueva = s_causa;
    cerrar_periodo(false);                /* el periodo interrumpido, como parcial */
    s_causa = causa_nueva;
  }
  almacen_estado(&est);
  report_almacen(&est, recuperado);

#if BOLO_DIAG_I2C
  escaneo_i2c();
#endif

  s_acc_ok = acc_init(hi2c);
  report_sensor(ACC_NOMBRE, s_acc_ok, acc_id());
  s_temp_ok = temp_init(hi2c);
  report_sensor(TEMP_NOMBRE, s_temp_ok, s_temp_ok ? TEMP_I2C_ADDR : 0U);

  s_radio_ok = radio_probar();            /* si falla, el bolo sigue midiendo igual */

  azar_semilla(reloj_ahora() ^ RTC->SSR ^ (HAL_GetTick() << 16));

  /* Primer periodo: arranca en la epoca actual (parcial) */
  uint32_t t = reloj_ahora();
  abrir_periodo((t / T_EPOCA_S) * T_EPOCA_S);
  s_seg       = t;
  s_minuto_id = t / T_SHT_S;

  /* Primer contacto */
  if (s_p.ranura_valida)
  {
    s_ciclo   = t / T_PERIODO_S;
    s_intento = 0U;
    planificar();
  }
  else
  {
    s_t_contacto = t + azar_rango(T_ARRANQUE_MIN_S, T_ARRANQUE_MAX_S);
    report_proximo(t, s_t_contacto);
  }
}

void app_run(void)
{
  servicio();

  if (!s_en_contacto && (int32_t)(reloj_ahora() - s_t_contacto) >= 0) hacer_contacto();

  power_dormir();   /* STOP2 hasta el proximo tick, muestra o fin de medicion */
}
