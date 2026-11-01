/**
 * @file    app_gw.c
 * @brief   Gateway del prototipo. Escucha LoRa en forma continua y muestra en
 *          PuTTY cada paquete con su RSSI y SNR, y cuenta los perdidos.
 *          Etapa 3: desarma el paquete de datos del bolo (bolo_packet.h) y lo
 *          muestra legible. Sigue entendiendo los "PING n" de la etapa 2.
 *          Resumen estadistico (para las mediciones): cada RESUMEN_CADA paquetes
 *          o con la tecla 'r' en PuTTY; la tecla 'c' empieza una medicion nueva.
 */
#include "app_gw.h"
#include "radio.h"
#include "bolo_packet.h"
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>

#define UART_TIMEOUT_MS   10U
#define LED_PULSO_MS      80U       /* el LED destella al llegar cada paquete  */
#define AVISO_SILENCIO_MS 30000U    /* si no llega nada en 30 s, se avisa      */
#define RESUMEN_CADA      20U       /* resumen automatico cada 20 paquetes     */

static UART_HandleTypeDef *s_huart = NULL;
static bool                s_radio_ok = false;

/* Estadisticas de la recepcion */
static uint32_t s_recibidos  = 0;    /* paquetes que llegaron bien            */
static uint32_t s_rotos      = 0;    /* llegaron con error de CRC/encabezado   */
static uint32_t s_perdidos   = 0;    /* saltos en la numeracion de los PING    */
static bool     s_hay_previo = false;
static uint32_t s_ultimo_n   = 0;    /* numero del ultimo paquete recibido     */

static uint32_t s_t_ultimo   = 0;    /* cuando llego el ultimo paquete [ms]    */
static uint32_t s_t_aviso    = 0;    /* cuando se aviso el ultimo silencio     */
static uint32_t s_t_led      = 0;    /* cuando se prendio el LED               */

/* Estadisticas de la medicion en curso (desde el arranque o desde la tecla 'c').
 * El RSSI varia 15-20 dB de un paquete a otro por los rebotes: para comparar
 * condiciones (distancia, agua, SF) se comparan PROMEDIOS, no paquetes sueltos. */
typedef struct
{
  uint32_t recibidos;
  uint32_t perdidos;
  uint32_t rotos;
  int32_t  rssi_suma;         /* [dBm]        */
  int16_t  rssi_min;
  int16_t  rssi_max;
  int32_t  snr_suma_x4;       /* [dB x 4]     */
  uint32_t t_inicio;          /* [ms]         */
} medicion_t;

static medicion_t s_med;

/* ---------- Puente printf -> UART (igual que en report.c del TX) ---------- */
int __io_putchar(int ch)
{
  if (s_huart != NULL)
  {
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(s_huart, &c, 1, UART_TIMEOUT_MS);
  }
  return ch;
}

/* ---------- Informe de la radio (mismo formato que en el TX) ---------- */
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

static void radio_informe(const radio_diag_t *r)
{
  printf("  [%s] SX1262 (estado 0x%02X = %s)\r\n",
         r->ok ? " OK  " : "ERROR", r->estado, radio_modo_str(r->estado));
  radio_errores("errores al arrancar", r->err_arranque);
  radio_errores("errores al final   ", r->err_final);
  if (!r->ok)
  {
    printf("          fallo en el paso %u: %s\r\n",
           (unsigned)r->paso_fallido, radio_paso_str(r->paso_fallido));
  }
}

/* ---------- Ayudas ---------- */
/* Imprime el SNR (que viene x4) con dos decimales sin usar float: 38 -> "9.50" */
static void imprimir_snr(int8_t snr_x4)
{
  int v = snr_x4;
  if (v < 0) { printf("-"); v = -v; }
  printf("%d.%02d", v / 4, (v % 4) * 25);
}

/* Imprime un entero en "unidades fijas" sin float: (2396, 2) -> "23.96".
 * Con 'con_signo' pone "+" a los positivos (para el delta). */
static void imprimir_fijo(int32_t v, uint8_t decimales, bool con_signo)
{
  int32_t div = (decimales == 2U) ? 100 : (decimales == 1U) ? 10 : 1;
  int32_t a   = (v < 0) ? -v : v;

  if (v < 0)          printf("-");
  else if (con_signo) printf("+");
  if (decimales == 0U) printf("%ld", (long)a);
  else printf("%ld.%0*ld", (long)(a / div), (int)decimales, (long)(a % div));
}

/* Cuenta los perdidos a partir del numero de secuencia. Detecta el reinicio del
 * bolo (la numeracion vuelve atras) y la vuelta del contador de 16 bits. */
static void contar_secuencia(uint32_t n, uint32_t modulo, const char *causa)
{
  if (s_hay_previo)
  {
    if (n > s_ultimo_n)
    {
      s_perdidos  += n - s_ultimo_n - 1U;
      s_med.perdidos += n - s_ultimo_n - 1U;
    }
    else if (modulo != 0U && s_ultimo_n > (modulo - 1000U) && n < 1000U)
    {
      s_perdidos  += (modulo - 1U - s_ultimo_n) + n;         /* el contador dio la vuelta */
      s_med.perdidos += (modulo - 1U - s_ultimo_n) + n;
    }
    else
    {
      printf("  (el bolo se reinicio: la numeracion volvio a %lu", (unsigned long)n);
      if (causa != NULL) printf(" | causa: %s", causa);
      printf(")\r\n");
    }
  }
  else if (causa != NULL)
  {
    printf("  (primer paquete del bolo | su ultimo reinicio fue por: %s)\r\n", causa);
  }
  s_ultimo_n   = n;
  s_hay_previo = true;
}

/* Si el texto es "PING <numero>", devuelve el numero en *n */
static bool leer_ping(const radio_rx_t *p, uint32_t *n)
{
  static const char pre[] = "PING ";
  uint32_t valor = 0;
  uint8_t  i;

  if (p->largo <= 5U) return false;
  for (i = 0; i < 5U; i++) if (p->datos[i] != (uint8_t)pre[i]) return false;
  for (i = 5U; i < p->largo; i++)
  {
    if (p->datos[i] < '0' || p->datos[i] > '9') return false;
    valor = valor * 10U + (uint32_t)(p->datos[i] - '0');
  }
  *n = valor;
  return true;
}

/* ---------- Estadisticas de la medicion ---------- */
static void medicion_borrar(void)
{
  s_med = (medicion_t){ 0 };
  s_med.t_inicio = HAL_GetTick();
}

static void medicion_sumar(const radio_rx_t *p)
{
  if (s_med.recibidos == 0U || p->rssi_dbm < s_med.rssi_min) s_med.rssi_min = p->rssi_dbm;
  if (s_med.recibidos == 0U || p->rssi_dbm > s_med.rssi_max) s_med.rssi_max = p->rssi_dbm;
  s_med.recibidos++;
  s_med.rssi_suma   += p->rssi_dbm;
  s_med.snr_suma_x4 += p->snr_x4;
}

/* Promedio entero redondeado (tambien con negativos) */
static int32_t promedio(int32_t suma, uint32_t n)
{
  int32_t m = (int32_t)n;
  return (suma >= 0) ? (suma + m / 2) / m : -((-suma + m / 2) / m);
}

static void medicion_resumen(void)
{
  uint32_t esperados = s_med.recibidos + s_med.perdidos;
  uint32_t seg       = (HAL_GetTick() - s_med.t_inicio) / 1000U;

  printf("\r\n  === Resumen (%lu s) | %lu recibidos | %lu perdidos (",
         (unsigned long)seg, (unsigned long)s_med.recibidos, (unsigned long)s_med.perdidos);
  imprimir_fijo((esperados > 0U) ? (int32_t)((s_med.perdidos * 1000U + esperados / 2U) / esperados) : 0,
                1U, false);
  printf(" %%) | %lu rotos\r\n", (unsigned long)s_med.rotos);

  if (s_med.recibidos > 0U)
  {
    printf("      RSSI promedio %ld dBm (min %d, max %d) | SNR promedio ",
           (long)promedio(s_med.rssi_suma, s_med.recibidos), s_med.rssi_min, s_med.rssi_max);
    imprimir_fijo(promedio(s_med.snr_suma_x4 * 25, s_med.recibidos), 2U, false);   /* x4 -> centesimas */
    printf(" dB ===\r\n\r\n");
  }
  else
  {
    printf("      (todavia no llego ningun paquete en esta medicion) ===\r\n\r\n");
  }
}

/* Tecla apretada en PuTTY (sin esperar): -1 si no hay ninguna */
static int leer_tecla(void)
{
  if (s_huart == NULL) return -1;
  if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(s_huart);
  if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE)) return (int)(s_huart->Instance->RDR & 0xFFU);
  return -1;
}

/* Encabezado comun de cada paquete recibido: calidad de la señal y perdidos */
static void imprimir_cabecera_rx(void)
{
  printf("  RX #%-4lu | ", (unsigned long)s_recibidos);
}

static void imprimir_senal(const radio_rx_t *p)
{
  printf(" | RSSI %d dBm | SNR ", p->rssi_dbm);
  imprimir_snr(p->snr_x4);
  printf(" dB | perdidos: %lu\r\n", (unsigned long)s_perdidos);
}

/* Paquete de datos del bolo (etapa 3): se muestra en 2-4 lineas */
static void mostrar_datos(const radio_rx_t *r, const bolo_pkt_t *d)
{
  contar_secuencia(d->seq, 65536U, bolo_pkt_rst_str(d->reinicio));

  /* Linea 1: quien, que numero y como llego */
  imprimir_cabecera_rx();
  printf("nodo %u | seq %u | %u B", d->nodo, d->seq, r->largo);
  imprimir_senal(r);

  /* Linea 2: lo que midio */
  printf("           %s | ejes ", bolo_pkt_mov_str(d->mov));
  imprimir_fijo(d->desvio_ejes_dmg, 1U, false);
  printf(" mg | mag ");
  imprimir_fijo(d->desvio_mag_dmg, 1U, false);
  printf(" mg");
  if (d->temp_ok)
  {
    printf(" | T ");
    imprimir_fijo(d->temp_centi, 2U, false);
    printf(" C | delta ");
    imprimir_fijo(d->delta_centi, 2U, true);
    printf(" C (%s) | H %u %%", bolo_pkt_term_str(d->term), d->hum_pct);
  }
  else
  {
    printf(" | T --");
  }
  printf("\r\n");

  /* Linea 3 (si termino un evento termico en este ciclo) */
  if (d->evento_fin)
  {
    printf("           EVENTO: %s terminado | %u ciclos | %s ",
           d->evento_suba ? "suba (demo)" : "toma de agua?",
           d->ev_ciclos, d->evento_suba ? "maximo" : "minimo");
    imprimir_fijo(d->ev_extremo_centi, 2U, true);
    printf(" C\r\n");
  }

  /* Linea 4 (si hubo problemas en el bolo) */
  if (!d->temp_ok || d->ventana_cortada || d->bus_recuperado)
  {
    printf("           avisos:%s%s%s\r\n",
           d->temp_ok         ? "" : " temperatura no disponible;",
           d->ventana_cortada ? " ventana del acelerometro cortada;" : "",
           d->bus_recuperado  ? " bus I2C reiniciado;" : "");
  }
}

/* Paquete de texto (los PING de la etapa 2) */
static void mostrar_texto(const radio_rx_t *p)
{
  uint32_t n;

  if (leer_ping(p, &n)) contar_secuencia(n, 0U, NULL);

  imprimir_cabecera_rx();
  printf("\"");
  for (uint8_t i = 0; i < p->largo; i++)
  {
    uint8_t c = p->datos[i];
    printf("%c", (c >= 32U && c < 127U) ? (char)c : '.');
  }
  printf("\" (%u B)", p->largo);
  imprimir_senal(p);
}

/* Muestra un paquete recibido y actualiza las estadisticas */
static void mostrar_paquete(const radio_rx_t *p)
{
  bolo_pkt_t d;

  if (!p->crc_ok)
  {
    s_rotos++;
    s_med.rotos++;
    printf("  RX roto  | descartado (irq 0x%04X) | RSSI %d dBm | SNR ", p->irq, p->rssi_dbm);
    imprimir_snr(p->snr_x4);
    printf(" dB | rotos: %lu\r\n", (unsigned long)s_rotos);
    return;
  }

  s_recibidos++;

  if (bolo_pkt_decode(p->datos, p->largo, &d)) mostrar_datos(p, &d);
  else                                         mostrar_texto(p);

  medicion_sumar(p);
  if ((s_med.recibidos % RESUMEN_CADA) == 0U) medicion_resumen();
}

/* ===================== Funciones publicas ===================== */
void app_init(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi)
{
  radio_diag_t diag;

  s_huart = huart;

  printf("\r\n==================================================\r\n");
  printf("  BOLO PROTOTIPO - Gateway (nodo RX)          v0.4\r\n");
  printf("==================================================\r\n");

  s_radio_ok = radio_init(hspi, &diag);
  radio_informe(&diag);

  if (s_radio_ok && radio_rx_start())
  {
    printf("  Escuchando en 915 MHz (SF7, 125 kHz, 4/5)...\r\n");
    printf("  Teclas: r = resumen ahora | c = borrar estadisticas (medicion nueva)\r\n");
  }
  else
  {
    s_radio_ok = false;
    printf("  No se pudo poner la radio a escuchar.\r\n");
  }

  s_t_ultimo = HAL_GetTick();
  s_t_aviso  = s_t_ultimo;
  medicion_borrar();
}

void app_run(void)
{
  radio_rx_t pkt;
  uint32_t   ahora = HAL_GetTick();

  if (!s_radio_ok)
  {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);   /* parpadeo rapido = radio con falla */
    HAL_Delay(200);
    return;
  }

  /* ¿Llego algo? */
  if (radio_rx_poll(&pkt))
  {
    mostrar_paquete(&pkt);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    s_t_led    = ahora;
    s_t_ultimo = ahora;
    s_t_aviso  = ahora;
  }

  /* El LED se apaga solo despues del destello */
  if ((ahora - s_t_led) > LED_PULSO_MS)
  {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  }

  /* Aviso de silencio: ayuda a darse cuenta de que el TX no llega */
  if ((ahora - s_t_aviso) > AVISO_SILENCIO_MS)
  {
    printf("  ... sin paquetes hace %lu s\r\n", (unsigned long)((ahora - s_t_ultimo) / 1000U));
    s_t_aviso = ahora;
  }

  /* Teclas de PuTTY */
  switch (leer_tecla())
  {
    case 'r': case 'R':
      medicion_resumen();
      break;
    case 'c': case 'C':
      medicion_resumen();                          /* lo que se cierra */
      medicion_borrar();
      s_hay_previo = false;                        /* no contar el hueco como perdidos */
      printf("  --- Estadisticas borradas: empieza una medicion nueva ---\r\n\r\n");
      break;
    case -1:
      break;
    default:
      printf("  Teclas: r = resumen ahora | c = borrar estadisticas (medicion nueva)\r\n");
      break;
  }
}
