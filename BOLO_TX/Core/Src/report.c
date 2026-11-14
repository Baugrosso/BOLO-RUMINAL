/**
 * @file    report.c
 * @brief   Interfaz por UART (PuTTY) del BOLO_TX. Sin floats en printf (newlib-nano).
 */
#include "report.h"
#include "bolo_config.h"
#include "bolo_reloj.h"
#include <stdio.h>
#include <stdlib.h>

#define FW_VERSION        "2.0"
#define UART_TIMEOUT_MS   10U

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

#if (BOLO_MODO == BOLO_MODO_REAL)
  #define MODO_STR  "REAL (periodo 6 h, ranuras de 2 min)"
#else
  #define MODO_STR  "BANCO (periodo 30 min, ranuras de 10 s)"
#endif

#define LOG(nivel)  if (BOLO_LOG >= (nivel))

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
static void hora_str(uint32_t t_unix, char *s, size_t n)
{
  uint16_t a; uint8_t me, d, h, mi, se;
  reloj_de_unix(t_unix, &a, &me, &d, &h, &mi, &se);
  (void)snprintf(s, n, "%04u-%02u-%02u %02u:%02u:%02u", a, me, d, h, mi, se);
}

/* centesimas -> "-12.34" */
static void centi_str(int32_t v, char *s, size_t n)
{
  const char *signo = (v < 0) ? "-" : "";
  uint32_t a = (uint32_t)labs(v);
  (void)snprintf(s, n, "%s%lu.%02lu", signo, (unsigned long)(a / 100U), (unsigned long)(a % 100U));
}

/* ---------- Publicas ---------- */
void report_init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  (void)setvbuf(stdout, NULL, _IONBF, 0);     /* sin buffer: no usa heap */
}

void report_banner(void)
{
  LOG(1)
  {
    printf("\r\n" C_NEGRITA "========================================" C_RESET "\r\n");
    printf(C_NEGRITA " BOLO_TX v" FW_VERSION " - nodo %u" C_RESET "\r\n", (unsigned)BOLO_NODO_ID);
    printf(" Modo: %s\r\n", MODO_STR);
    printf(" Epocas de %us, temperatura cada %us (media de %u min)\r\n",
           (unsigned)T_EPOCA_S, (unsigned)T_SHT_S, (unsigned)(T_MEDIA_TH_S / 60U));
    printf(" LoRa SF9/125k, +%d dBm, salto de 64 canales\r\n", (int)RADIO_POTENCIA_DBM);
    printf(C_NEGRITA "========================================" C_RESET "\r\n");
  }
}

void report_reinicio(const char *causa, bool anomalo)
{
  LOG(1) printf(" Reinicio: %s%s" C_RESET "\r\n", anomalo ? C_ROJO : C_GRIS, causa);
}

void report_i2c(const uint8_t *dirs, uint8_t n)
{
  LOG(1)
  {
    printf(" Bus I2C:");
    if (n == 0U) printf(C_ROJO " sin dispositivos" C_RESET);
    for (uint8_t i = 0; i < n; i++) printf(" 0x%02X", dirs[i]);
    printf("\r\n");
  }
}

void report_crc(bool hw_ok)
{
  LOG(1)
  {
    if (hw_ok) printf(" CRC-32    " C_VERDE "OK" C_RESET " (unidad por hardware)\r\n");
    else       printf(" CRC-32    " C_AMARILLO "por software" C_RESET " (la unidad por hardware no dio la referencia)\r\n");
  }
}

void report_sensor(const char *nombre, bool ok, uint8_t id)
{
  LOG(1)
  {
    if (ok) printf(" %-9s " C_VERDE "OK" C_RESET " (0x%02X)\r\n", nombre, id);
    else    printf(" %-9s " C_ROJO "NO RESPONDE" C_RESET "\r\n", nombre);
  }
}

void report_radio(const radio_diag_t *r)
{
  LOG(1)
  {
    if (r->ok) printf(" Radio     " C_VERDE "OK" C_RESET " (%s)\r\n", radio_modo_str(r->estado));
    else       printf(" Radio     " C_ROJO "FALLA en '%s'" C_RESET " (errores 0x%04X)\r\n",
                      radio_paso_str(r->paso_fallido), (unsigned)r->err_final);
  }
}

void report_almacen(const alm_estado_t *e, bool recuperado)
{
  LOG(1)
  {
    printf(" Almacen: %u periodos sin ACK | nucleo %lu/%u B | aceleracion %lu/%u B%s\r\n",
           (unsigned)e->periodos, (unsigned long)e->uso_nucleo, (unsigned)ALM_NUCLEO_BYTES,
           (unsigned long)e->uso_acel, (unsigned)ALM_ACEL_BYTES,
           recuperado ? C_AMARILLO " (periodo en armado recuperado)" C_RESET : "");
  }
}

void report_hora(uint32_t t_unix, int32_t delta, bool primera)
{
  LOG(1)
  {
    char s[32];
    hora_str(t_unix, s, sizeof(s));
    printf(C_CIAN " Hora de la gateway: %s UTC (ajuste %+ld s)%s" C_RESET "\r\n",
           s, (long)delta, primera ? " - primera sincronizacion" : "");
  }
}

void report_epoca(uint32_t t, const epoca_t *e)
{
  LOG(2)
  {
    char s[32];
    hora_str(t, s, sizeof(s));
    if (e->desvio_cod == FMT_DESVIO_INVALIDO)
      printf(C_GRIS " %s epoca INVALIDA (%u muestras)" C_RESET "\r\n", s, (unsigned)e->n_muestras);
    else
      printf(" %s epoca n=%u  X=%d Y=%d Z=%d mg  desvio cod=%u\r\n", s, (unsigned)e->n_muestras,
             (int)e->x_mg, (int)e->y_mg, (int)e->z_mg, (unsigned)e->desvio_cod);
  }
}

void report_th(uint32_t t, bool ok, int16_t t_centi, uint16_t h_centi)
{
  LOG(2)
  {
    char s[32], ts[12], hs[12];
    hora_str(t, s, sizeof(s));
    if (!ok) { printf(C_ROJO " %s SHT31 sin lectura" C_RESET "\r\n", s); return; }
    centi_str(t_centi, ts, sizeof(ts));
    centi_str((int32_t)h_centi, hs, sizeof(hs));
    printf(" %s T=%s C  HR=%s %%\r\n", s, ts, hs);
  }
}

void report_periodo(const periodo_info_t *p, uint16_t vdd_mv, bool parcial)
{
  LOG(1)
  {
    char s[32];
    hora_str(p->marca, s, sizeof(s));
    printf(C_NEGRITA " Periodo #%u cerrado%s" C_RESET " desde %s: %u epocas (%u validas), %u medias T/HR\r\n",
           (unsigned)p->n_periodo, parcial ? " (parcial)" : "", s,
           (unsigned)p->n_epocas, (unsigned)p->n_validas, (unsigned)p->n_th);
    printf("   comprimido: nucleo %u B + aceleracion %u B | VDD %u mV %s\r\n",
           (unsigned)p->bytes_nucleo, (unsigned)p->bytes_acel, (unsigned)vdd_mv,
           p->guardado ? "" : C_ROJO "| NO SE PUDO GUARDAR" C_RESET);
  }
}

void report_contacto_inicio(uint8_t contacto, uint8_t intento, bool pide_ranura)
{
  LOG(1)
  {
    char s[32];
    hora_str(reloj_ahora(), s, sizeof(s));
    printf(C_CIAN " %s Contacto #%u (intento %u)%s" C_RESET "\r\n", s, (unsigned)contacto,
           (unsigned)intento + 1U, pide_ranura ? " - pide hora y ranura" : "");
  }
}

void report_contacto_fin(const cont_res_t *r)
{
  LOG(1)
  {
    static const char *const k_estado[] = { "ACK", "sin te oigo", "sin ACK", "falla de radio" };
    const char *col = (r->estado == CONT_OK) ? C_VERDE : C_ROJO;
    printf("   %s%s" C_RESET ": %u periodos, %u fragmentos, %u reenviados en %u rondas, %u borrados, %lu ms\r\n",
           col, k_estado[r->estado], (unsigned)r->periodos, (unsigned)r->fragmentos,
           (unsigned)r->reenviados, (unsigned)r->rondas, (unsigned)r->borrados,
           (unsigned long)r->duracion_ms);
    if (r->estado == CONT_OK || r->estado == CONT_SIN_ACK)
    {
      printf("   te oigo: RSSI %d dBm SNR %d/4 dB", (int)r->rssi_teoigo, (int)r->snr_teoigo_x4);
      if (r->estado == CONT_OK)
        printf(" | ACK: RSSI %d dBm SNR %d/4 dB%s", (int)r->rssi_ack, (int)r->snr_ack_x4,
               r->completo ? " (completo)" : " (parcial)");
      printf("\r\n");
    }
  }
}

void report_ranura(uint8_t ranura)
{
  LOG(1) printf(C_CIAN "   Ranura asignada: %u (%u s despues de cada periodo)" C_RESET "\r\n",
                (unsigned)ranura, (unsigned)(ranura * T_RANURA_S));
}

void report_potencia(int8_t dbm)
{
  LOG(1) printf(C_CIAN "   Potencia de TX: %d dBm" C_RESET "\r\n", (int)dbm);
}

void report_proximo(uint32_t ahora, uint32_t t_contacto)
{
  LOG(1)
  {
    char s[32];
    hora_str(t_contacto, s, sizeof(s));
    printf(C_GRIS "   Proximo contacto: %s (en %lu s)" C_RESET "\r\n", s,
           (unsigned long)((t_contacto > ahora) ? (t_contacto - ahora) : 0U));
  }
}
