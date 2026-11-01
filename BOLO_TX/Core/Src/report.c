/**
 * @file    report.c
 * @brief   Interfaz por UART (PuTTY) del prototipo BOLO_TX.
 */
#include "report.h"
#include "bolo_config.h"
#include <stdio.h>
#include <stddef.h>

#define FW_VERSION        "0.1"
#define UART_TIMEOUT_MS   10U

/* ---------- Colores ANSI (PuTTY los interpreta) ---------- */
#if BOLO_UART_COLOR
  #define C_VERDE     "\033[32m"
  #define C_AMARILLO  "\033[33m"
  #define C_ROJO      "\033[31m"
  #define C_GRIS      "\033[90m"
  #define C_CIAN      "\033[36m"
  #define C_NEGRITA   "\033[1m"
  #define C_RESET     "\033[0m"
#else
  #define C_VERDE     ""
  #define C_AMARILLO  ""
  #define C_ROJO      ""
  #define C_GRIS      ""
  #define C_CIAN      ""
  #define C_NEGRITA   ""
  #define C_RESET     ""
#endif

#if (BOLO_MODO == BOLO_MODO_DEMO)
  #define MODO_STR  "DEMO"
#else
  #define MODO_STR  "PRUEBA"
#endif

static UART_HandleTypeDef *s_huart = NULL;

/* ---------- Puente printf -> UART ---------- */
int __io_putchar(int ch)
{
  if (s_huart != NULL)
  {
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(s_huart, &c, 1, UART_TIMEOUT_MS);
  }
  return ch;
}

/* ---------- Auxiliares ---------- */
const char *report_estado_str(bolo_estado_t e)
{
  switch (e)
  {
    case BOLO_ESTADO_REPOSO:     return "REPOSO";
    case BOLO_ESTADO_MOV_SUAVE:  return "MOVIMIENTO SUAVE";
    case BOLO_ESTADO_MOV_FUERTE: return "MOVIMIENTO FUERTE";
    default:                     return "DESCONOCIDO";
  }
}

static const char *estado_color(bolo_estado_t e)
{
  switch (e)
  {
    case BOLO_ESTADO_REPOSO:     return C_VERDE;
    case BOLO_ESTADO_MOV_SUAVE:  return C_AMARILLO;
    case BOLO_ESTADO_MOV_FUERTE: return C_ROJO;
    default:                     return C_GRIS;
  }
}

static const char *term_str(term_estado_t e)
{
  switch (e)
  {
    case TERM_APRENDIENDO: return "aprendiendo";
    case TERM_NORMAL: return "normal";
    case TERM_BAJADA: return "BAJADA (posible toma de agua)";
    case TERM_SUBA:   return "SUBA";
    default:          return "sin dato";
  }
}

static const char *term_color(term_estado_t e)
{
  switch (e)
  {
    case TERM_BAJADA: return C_CIAN;
    case TERM_SUBA:   return C_ROJO;
    case TERM_NORMAL: return C_VERDE;
    default:          return C_GRIS;
  }
}

static bool term_es_evento(term_estado_t e)
{
  return (e == TERM_BAJADA || e == TERM_SUBA);
}

#if BOLO_UART_DETALLE
/* Resumen del evento que termino en este ciclo */
static void report_evento_fin(const bolo_data_t *d)
{
  bool     baj = (d->term_fin_tipo == TERM_BAJADA);
  uint32_t seg = (uint32_t)d->term_fin_ciclos * T_CICLO_S;

  printf("  Evento   : %s%s%s %s | %u ciclos (~%lu %s) | %s %+.2f C\r\n",
         term_color(d->term_fin_tipo), baj ? "toma de agua?" : "suba (demo)", C_RESET,
         d->term_fin_por_tope ? "cortada" : "terminada",
         d->term_fin_ciclos, (unsigned long)((seg < 120U) ? seg : seg / 60U),
         (seg < 120U) ? "s" : "min",
         baj ? "minimo" : "maximo", d->term_fin_extremo_centi / 100.0f);
}
#endif

/* ---------- Funciones publicas ---------- */
void report_init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
}

void report_banner(void)
{
  printf("\r\n" C_NEGRITA "==================================================" C_RESET "\r\n");
  printf("  BOLO PROTOTIPO - Nodo TX                  v" FW_VERSION "\r\n");
  printf("  Modo %s | Reposo %u s | Sensado %u s a %u Hz\r\n",
         MODO_STR, T_REPOSO_S, T_SENSADO_S, ACC_FS_HZ);
  printf("  Umbrales: suave %u mg (ejes) | fuerte %u mg (magnitud)\r\n",
         MOV_UMBRAL_SUAVE_MG, MOV_UMBRAL_FUERTE_MG);
  printf(C_NEGRITA "==================================================" C_RESET "\r\n");
}

void report_i2c(const uint8_t *dirs, uint8_t n)
{
  printf("  Escaneo I2C:");
  if (n == 0) printf(" " C_ROJO "ningun dispositivo" C_RESET);
  for (uint8_t i = 0; i < n; i++) printf(" 0x%02X", dirs[i]);
  printf("\r\n");
}

void report_reinicio(const char *causa, bool anomalo)
{
  printf("  Reinicio: %s%s%s\r\n", anomalo ? C_ROJO C_NEGRITA : C_GRIS, causa, C_RESET);
}

void report_prueba_falla(uint8_t tipo)
{
  if (tipo == 1U)
    printf(C_AMARILLO "[PRUEBA] cuelgue a proposito: el watchdog deberia resetear en ~25 s" C_RESET "\r\n");
  else
    printf(C_AMARILLO "[PRUEBA] HardFault a proposito: deberia resetear en el acto" C_RESET "\r\n");
}

/* Imprime un registro de errores del SX1262: valor y nombre de cada bit prendido */
static void radio_errores(const char *titulo, uint16_t e)
{
  printf("          %s: 0x%04X", titulo, e);
  if (e == 0U) printf(" (ninguno)");
  for (uint8_t b = 0; b < 16U; b++)
  {
    if ((e & (1U << b)) != 0U)
    {
      const char *n = radio_error_str(b);
      printf(" %s", (n != NULL) ? n : "?");
    }
  }
  printf("\r\n");
}


void report_radio(const radio_diag_t *r)
{
  if (r == NULL) return;

  printf("  %s[%s]%s SX1262 (estado 0x%02X = %s)\r\n",
         r->ok ? C_VERDE : C_ROJO, r->ok ? " OK  " : "ERROR", C_RESET,
         r->estado, radio_modo_str(r->estado));
  radio_errores("errores al arrancar", r->err_arranque);
  radio_errores("errores al final   ", r->err_final);
  if (!r->ok)
  {
    printf("          " C_ROJO "fallo en el paso %u: %s" C_RESET "\r\n",
           (unsigned)r->paso_fallido, radio_paso_str(r->paso_fallido));
  }
}
void report_radio_tx(const char *msg, const radio_tx_t *t)
{
  if (msg == NULL || t == NULL) return;

  if (t->res == RADIO_TX_OK)
  {
    printf("  LoRa TX  : \"%s\" -> " C_VERDE "OK" C_RESET " (%lu ms en el aire)\r\n",
           msg, (unsigned long)t->ms);
  }
  else
  {
    printf("  LoRa TX  : \"%s\" -> " C_ROJO "ERROR: %s" C_RESET " (irq 0x%04X, %lu ms)\r\n",
           msg, radio_tx_str(t->res), t->irq, (unsigned long)t->ms);
  }
}

void report_init_sensor(const char *nombre, bool ok, uint8_t id)
{
  printf("  %s[%s]%s %s ", ok ? C_VERDE : C_ROJO, ok ? " OK  " : "ERROR", C_RESET, nombre);
  if (ok)            printf("(ID 0x%02X)\r\n", id);
  else if (id == 0U) printf("(sin respuesta)\r\n");
  else               printf("(ID 0x%02X: no es el esperado)\r\n", id);
}

void report_sensando(uint32_t ciclo)
{
#if BOLO_UART_DETALLE
  printf("\r\n" C_GRIS "------------------- Ciclo #%lu -------------------" C_RESET "\r\n",
         (unsigned long)ciclo);
  printf("[SENSANDO] %u s ... " C_NEGRITA "mover ahora" C_RESET "\r\n", T_SENSADO_S);
#else
  (void)ciclo;
#endif
}

void report_resultado(const bolo_data_t *d)
{
  if (d == NULL) return;

#if BOLO_UART_DETALLE
  printf("  Muestras : %u/%u", d->n_muestras, ACC_N_MUESTRAS);
  if (d->fallas_lectura > 0U || d->ventana_cortada)
  {
    printf("  " C_ROJO "(fallas: %u%s)" C_RESET, d->fallas_lectura,
           d->ventana_cortada ? ", ventana cortada" : "");
  }
  printf("\r\n");
  printf("  Ventana  : %u ms\r\n", d->ventana_ms);
  if (d->n_muestras == 0U)
  {
    printf("  Acelerom.: " C_ROJO "sin datos" C_RESET " (ninguna lectura valida)\r\n");
  }
  else
  {
    printf("  Media    : X=%6.2f  Y=%6.2f  Z=%6.2f  g\r\n",
           d->media.x, d->media.y, d->media.z);
    printf("  Desvio   : X=%6.1f  Y=%6.1f  Z=%6.1f  mg   (max %.1f mg)\r\n",
           d->desvio.x * 1000.0f, d->desvio.y * 1000.0f, d->desvio.z * 1000.0f,
           d->desvio_max * 1000.0f);
    printf("  Magnitud : desvio %6.1f mg\r\n", d->desvio_mag * 1000.0f);
  }
  if (d->temp_ok)
  {
    printf("  Temp.    : %5.2f C   | Humedad interna: %4.1f %%\r\n",
           d->temp_centi / 100.0f, d->hum_centi / 100.0f);
    printf("  Termica  : base %5.2f C | delta %+5.2f C | %s%s%s",
           d->temp_base_centi / 100.0f, d->temp_delta_centi / 100.0f,
           term_color(d->term_estado), term_str(d->term_estado), C_RESET);
    if (d->term_estado == TERM_APRENDIENDO)
      printf(" %u/%u", d->term_ciclos_evento, TERM_APRENDIZAJE_CICLOS);
    if (term_es_evento(d->term_estado))
      printf(" (ciclo %u del evento)", d->term_ciclos_evento);
    if (d->term_fin_por_tope)
      printf(" " C_AMARILLO "(base reaprendida)" C_RESET);
    printf("\r\n");
    if (term_es_evento(d->term_fin_tipo)) report_evento_fin(d);
  }
  else
  {
    printf("  Temp.    : " C_ROJO "no disponible" C_RESET "\r\n");
  }
  if (d->bus_recuperado)
  {
    printf("  Bus I2C  : " C_AMARILLO "destrabado y reiniciado" C_RESET
           " (los sensores que fallaron se reintentan el proximo ciclo)\r\n");
  }
  printf("  >>> ESTADO: %s%s%s\r\n",
         estado_color(d->estado), report_estado_str(d->estado), C_RESET);
#else
  printf("#%lu %s%s%s | ejes %.1f mg | mag %.1f mg | n=%u",
         (unsigned long)d->ciclo, estado_color(d->estado),
         report_estado_str(d->estado), C_RESET,
         d->desvio_max * 1000.0f, d->desvio_mag * 1000.0f, d->n_muestras);
  if (d->temp_ok)
  {
    printf(" | T %.2f C (%+.2f)", d->temp_centi / 100.0f, d->temp_delta_centi / 100.0f);
    if (term_es_evento(d->term_estado) || d->term_estado == TERM_APRENDIENDO)
      printf(" %s%s%s", term_color(d->term_estado), term_str(d->term_estado), C_RESET);
    if (term_es_evento(d->term_fin_tipo))
      printf(" | fin %s %uc %+.2f", (d->term_fin_tipo == TERM_BAJADA) ? "toma" : "suba",
             d->term_fin_ciclos, d->term_fin_extremo_centi / 100.0f);
  }
  else printf(" | T --");
  if (d->fallas_lectura > 0U || d->ventana_cortada)
  {
    printf(" | " C_ROJO "fallas %u%s" C_RESET, d->fallas_lectura,
           d->ventana_cortada ? " CORTADA" : "");
  }
  if (d->bus_recuperado) printf(" | bus reiniciado");
  printf("\r\n");
#endif
}

void report_durmiendo(uint32_t segundos)
{
#if BOLO_UART_DETALLE
  printf(C_GRIS "[DURMIENDO] proximo sensado en %lu s" C_RESET "\r\n", (unsigned long)segundos);
#else
  (void)segundos;
#endif
}
