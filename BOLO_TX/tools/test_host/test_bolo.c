/**
 * Pruebas en la PC de la logica del BOLO_TX (sin hardware).
 *
 * Compila los mismos .c del firmware con -DBOLO_HOST y prueba:
 *   - compresion: listas aleatorias, peor caso, casos borde (lo verifica Python)
 *   - motion: medias y codigo log del desvio
 *   - periodo + almacen + vista de transmision: periodos completos y parciales,
 *     respaldo y recuperacion despues de un "reset", reestampado de hora,
 *     borrado por ACK, llenado de las zonas y compactacion
 *   - reloj: conversion calendario <-> UNIX
 *
 * Escribe en el directorio de salida:
 *   rafaga.txt  fragmentos de un contacto (hex, como los reenvia la gateway)
 *   verdad.json los valores que tienen que salir al decodificar
 *   listas.txt  listas sueltas para probar el decodificador de compresion
 * y tools/test_host/verificar.py compara todo.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "bolo_config.h"
#include "compresion.h"
#include "motion.h"
#include "periodo.h"
#include "almacen.h"
#include "bolo_packet.h"
#include "contacto.h"
#include "bolo_reloj.h"
#include "bolo_util.h"
#include "salto.h"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FALLA %s:%d: %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

static uint32_t s_rng = 12345U;
static uint32_t rnd(void) { s_rng ^= s_rng << 13; s_rng ^= s_rng >> 17; s_rng ^= s_rng << 5; return s_rng; }
static int32_t rnd_rango(int32_t a, int32_t b) { return a + (int32_t)(rnd() % (uint32_t)(b - a + 1)); }

/* ------------------------------------------------------------------ verdad */
typedef struct
{
  uint16_t n_periodo;
  uint32_t marca;
  int      n_ep, n_th;
  int16_t  temp[MEDIAS_TH_POR_PERIODO];
  uint8_t  hum[MEDIAS_TH_POR_PERIODO];
  uint8_t  desv[EPOCAS_POR_PERIODO];
  int16_t  x[EPOCAS_POR_PERIODO], y[EPOCAS_POR_PERIODO], z[EPOCAS_POR_PERIODO];
  int      sin_acel;                       /* la aceleracion se piso */
} verdad_t;

static void json_lista_i(FILE *f, const char *nom, const int16_t *v, int n, const uint8_t *filtro)
{
  fprintf(f, "\"%s\":[", nom);
  int primero = 1;
  for (int i = 0; i < n; i++)
  {
    if (filtro && filtro[i] == FMT_DESVIO_INVALIDO) continue;
    fprintf(f, "%s%d", primero ? "" : ",", v[i]); primero = 0;
  }
  fprintf(f, "]");
}

static void json_lista_u(FILE *f, const char *nom, const uint8_t *v, int n)
{
  fprintf(f, "\"%s\":[", nom);
  for (int i = 0; i < n; i++) fprintf(f, "%s%u", i ? "," : "", v[i]);
  fprintf(f, "]");
}

static void json_verdad(FILE *f, const verdad_t *v, int coma)
{
  int ne = v->sin_acel ? 0 : v->n_ep;
  fprintf(f, "{\"n_periodo\":%u,\"marca\":%u,\"n_epocas\":%d,\"n_th\":%d,",
          v->n_periodo, v->marca, ne, v->n_th);
  json_lista_i(f, "temp", v->temp, v->n_th, NULL); fprintf(f, ",");
  json_lista_u(f, "hum", v->hum, v->n_th); fprintf(f, ",");
  json_lista_u(f, "desvio", v->desv, ne); fprintf(f, ",");
  json_lista_i(f, "x", v->x, ne, v->desv); fprintf(f, ",");
  json_lista_i(f, "y", v->y, ne, v->desv); fprintf(f, ",");
  json_lista_i(f, "z", v->z, ne, v->desv);
  fprintf(f, "}%s\n", coma ? "," : "");
}

/* ------------------------------------------------------------------ armado */
/* Genera un periodo con datos sinteticos. 'peor' = valores al azar (incompresibles). */
static void generar(verdad_t *v, uint16_t n, uint32_t marca, int n_ep, int n_th, int peor)
{
  memset(v, 0, sizeof(*v));
  v->n_periodo = n; v->marca = marca; v->n_ep = n_ep; v->n_th = n_th;

  int32_t x = 20, y = -15, z = 1000, t = 3900;
  for (int i = 0; i < n_ep; i++)
  {
    if (peor) { x = rnd_rango(-2048, 2047); y = rnd_rango(-2048, 2047); z = rnd_rango(-2048, 2047); }
    else
    {
      /* vaca: casi siempre quieta, a veces se mueve */
      int mueve = (rnd() % 10U) == 0U;
      x += mueve ? rnd_rango(-300, 300) : rnd_rango(-2, 2);
      y += mueve ? rnd_rango(-300, 300) : rnd_rango(-2, 2);
      z += mueve ? rnd_rango(-300, 300) : rnd_rango(-2, 2);
      if (x > 2047) x = 2047;
      if (x < -2048) x = -2048;
      if (y > 2047) y = 2047;
      if (y < -2048) y = -2048;
      if (z > 2047) z = 2047;
      if (z < -2048) z = -2048;
    }
    v->x[i] = (int16_t)x; v->y[i] = (int16_t)y; v->z[i] = (int16_t)z;
    v->desv[i] = peor ? (uint8_t)rnd_rango(0, 254) : (uint8_t)rnd_rango(20, 40);
    if ((rnd() % 37U) == 0U) v->desv[i] = FMT_DESVIO_INVALIDO;     /* epocas invalidas */
  }
  for (int j = 0; j < n_th; j++)
  {
    t += peor ? rnd_rango(-20000, 20000) : rnd_rango(-8, 8);
    if (t < -32767) t = -32767;
    if (t > 32767) t = 32767;
    v->temp[j] = (int16_t)t;
    v->hum[j]  = peor ? (uint8_t)rnd_rango(0, 200) : (uint8_t)rnd_rango(96, 104);
    if (j == 5) { v->temp[j] = FMT_TEMP_SIN_DATO; v->hum[j] = FMT_HUM_SIN_DATO; }
  }
}

/* Pasa un periodo generado por periodo.c y lo guarda en el almacen.
 * Las temperaturas se meten como mediciones del SHT31 (1 por media) para pasar
 * por el mismo camino de redondeo del firmware. */
static void cargar(const verdad_t *v, int recuperar_a_mitad)
{
  periodo_abrir(v->marca, false, RST_WATCHDOG);
  int corte = recuperar_a_mitad ? v->n_ep / 2 : -1;

  for (int i = 0; i < v->n_ep; i++)
  {
    epoca_t e = { v->x[i], v->y[i], v->z[i], v->desv[i], 100 };
    CHECK(periodo_epoca(&e));
    int j = i / (T_MEDIA_TH_S / T_EPOCA_S);
    if ((i % (T_MEDIA_TH_S / T_EPOCA_S)) == (T_MEDIA_TH_S / T_EPOCA_S) - 1 && j < v->n_th)
    {
      if (v->temp[j] != FMT_TEMP_SIN_DATO)
        periodo_muestra_th(v->temp[j], (uint16_t)(v->hum[j] * 50U));
      CHECK(periodo_cerrar_th());
      periodo_respaldar();
    }
    if (i == corte)
    {
      /* "Reset": se respalda, se ensucia el estado agregando basura que no entra en la
       * foto, y se recupera. Tiene que volver exactamente a la foto. */
      periodo_respaldar();
      epoca_t basura = { 111, 222, 333, 44, 100 };
      for (int k = 0; k < 7; k++) CHECK(periodo_epoca(&basura));
      periodo_muestra_th(9999, 9999);
      CHECK(periodo_recuperar());
      CHECK(periodo_n_epocas() == (uint16_t)(i + 1));
    }
  }
  periodo_cierre_t c = { v->n_periodo, 214, 3 };   /* VDD 3,298 V, 3 contactos sin ACK */
  periodo_info_t info;
  periodo_cerrar(&c, &info);
  CHECK(info.guardado);
  CHECK(info.n_epocas == v->n_ep && info.n_th == v->n_th);
}

/* Fragmenta como contacto.c y escribe la rafaga (hex) */
static int escribir_rafaga(FILE *f, uint8_t nodo, uint8_t contacto, int max_per)
{
  alm_periodo_t p;
  uint16_t idx = 0;
  for (uint16_t k = 0; k < max_per && almacen_nuevo(k, &p); k++)
  {
    uint32_t largo = contacto_largo_periodo(p.largo_nucleo, p.largo_acel);
    uint32_t crc   = contacto_crc_periodo(p.n_periodo, p.largo_nucleo, p.largo_acel);
    for (uint32_t o = 0; o < largo; o += PKT_FRAG_DATOS, idx++)
    {
      uint8_t buf[PKT_FRAG_MAX];
      uint16_t n = (uint16_t)((largo - o) < PKT_FRAG_DATOS ? (largo - o) : PKT_FRAG_DATOS);
      pkt_frag_cab(buf, nodo, contacto, idx, false);
      CHECK(contacto_leer_periodo(p.n_periodo, p.largo_nucleo, p.largo_acel, crc, o,
                                  &buf[PKT_FRAG_CAB], n) == n);
      for (uint16_t i = 0; i < PKT_FRAG_CAB + n; i++) fprintf(f, "%02x", buf[i]);
      fprintf(f, "\n");
    }
  }
  return idx;
}

/* ------------------------------------------------------------------ pruebas */
static void prueba_compresion(const char *dir)
{
  char ruta[512];
  snprintf(ruta, sizeof(ruta), "%s/listas.txt", dir);
  FILE *f = fopen(ruta, "w"); CHECK(f);

  static uint8_t buf[4096];
  cmp_t c;
  struct { int n, w, s, modo; } casos[] = {
    {0, 12, 1, 0}, {1, 12, 1, 0}, {2, 8, 0, 0}, {32, 16, 1, 0}, {33, 16, 1, 0},
    {1080, 12, 1, 0}, {1080, 12, 1, 1}, {1080, 8, 0, 1}, {36, 16, 1, 1}, {1080, 12, 1, 2},
    {100, 16, 1, 3}, {500, 12, 1, 4},
  };
  for (size_t ci = 0; ci < sizeof(casos) / sizeof(casos[0]); ci++)
  {
    int n = casos[ci].n, w = casos[ci].w, s = casos[ci].s;
    int32_t mn = s ? -(1 << (w - 1)) : 0, mx = s ? (1 << (w - 1)) - 1 : (1 << w) - 1;
    uint16_t cap = (uint16_t)CMP_BYTES_MAX((uint32_t)n, (uint32_t)w);
    cmp_iniciar(&c, buf, cap, (uint8_t)w, s);
    fprintf(f, "%d %d %d", n, w, s);
    int32_t v = 0;
    for (int i = 0; i < n; i++)
    {
      switch (casos[ci].modo)
      {
        case 0: v += rnd_rango(-3, 3); break;                 /* suave         */
        case 1: v = rnd_rango(mn, mx); break;                 /* peor caso     */
        case 2: v = 7; break;                                 /* constante     */
        case 3: v = (i & 1) ? mn : mx; break;                 /* extremos      */
        case 4: v = (i % 50 == 0) ? rnd_rango(mn, mx) : v; break;
      }
      if (v < mn) v = mn;
      if (v > mx) v = mx;
      CHECK(cmp_agregar(&c, buf, v));
      fprintf(f, " %d", v);
    }
    uint16_t nb = cmp_cerrar(&c, buf);
    CHECK(!c.desborde && nb <= cap);
    fprintf(f, " |");
    for (uint16_t i = 0; i < nb; i++) fprintf(f, "%02x", buf[i]);
    fprintf(f, "\n");
  }
  fclose(f);
  printf("compresion: %zu listas escritas\n", sizeof(casos) / sizeof(casos[0]));
}

static void prueba_motion(void)
{
  /* codigo log: 0,1 mg -> 0; 2000 mg -> 254; pasos de ~4 % */
  CHECK(motion_desvio_codigo(0.0f) == 0U);
  CHECK(motion_desvio_codigo(0.1f) == 0U);
  CHECK(motion_desvio_codigo(2000.0f) == 254U);
  CHECK(motion_desvio_codigo(50000.0f) == 254U);
  float r = powf(20000.0f, 1.0f / 254.0f);
  for (int k = 1; k < 254; k++)
  {
    float s = 0.1f * powf(r, (float)k);
    CHECK(abs((int)motion_desvio_codigo(s) - k) <= 0);
  }

  /* Epoca quieta en Z = 1 g: medias exactas y desvio chico */
  motion_reset();
  for (int i = 0; i < 100; i++)
  {
    acc_raw_t m = { (int16_t)(i & 1), 0, (int16_t)(16384 + ((i & 1) ? 16 : -16)) };
    motion_add(&m);
  }
  epoca_t e; motion_epoca(&e);
  CHECK(e.n_muestras == 100 && e.x_mg == 0 && e.y_mg == 0 && e.z_mg == 1000);
  /* desvio de |a| = 16 cuentas = 0,977 mg -> codigo ~59 */
  float esperado = 16.0f * 1000.0f / 16384.0f;
  CHECK(abs((int)e.desvio_cod - (int)motion_desvio_codigo(esperado)) <= 1);

  /* Menos de 50 muestras: invalida */
  motion_reset();
  for (int i = 0; i < 49; i++) { acc_raw_t m = { 0, 0, 16384 }; motion_add(&m); }
  motion_epoca(&e);
  CHECK(e.desvio_cod == FMT_DESVIO_INVALIDO);

  /* Saturacion a 12 bits: -2 g da -2000 mg (entra), un eje fuera de escala se limita */
  motion_reset();
  for (int i = 0; i < 60; i++) { acc_raw_t m = { -32768, 32767, 0 }; motion_add(&m); }
  motion_epoca(&e);
  CHECK(e.x_mg == -2000 && e.y_mg == 2000);
  printf("motion: OK\n");
}

static void prueba_reloj(void)
{
  uint16_t a; uint8_t me, d, h, mi, s;
  CHECK(reloj_a_unix(2000, 1, 1, 0, 0, 0) == 946684800U);
  CHECK(reloj_a_unix(2026, 11, 14, 16, 15, 0) == 1794672900U);
  CHECK(reloj_a_unix(2028, 2, 29, 23, 59, 59) == 1835481599U);
  for (uint32_t t = 946684800U; t < 4102444800U; t += 86400U * 7U + 3671U)
  {
    reloj_de_unix(t, &a, &me, &d, &h, &mi, &s);
    CHECK(reloj_a_unix(a, me, d, h, mi, s) == t);
  }
  printf("reloj: OK\n");
}

static void prueba_paquetes(void)
{
  pkt_meta_t m = { 2, 1080, 0xBEEF, 0x6A3B2C10U, 36, 214, 4, 15, 3, 9 }, r;
  uint8_t b[PKT_META_LARGO];
  pkt_meta_armar(b, &m);
  pkt_meta_leer(b, &r);
  CHECK(memcmp(&m, &r, sizeof(m)) == 0);

  uint8_t c[4];
  pkt_frag_cab(c, 1, 200, 1023, true);
  CHECK(c[0] == 1 && c[1] == 200 && c[2] == 0xFF && c[3] == 0xE0);
  pkt_frag_cab(c, 2, 0, 5, false);
  CHECK(c[2] == 0x01 && c[3] == 0x40);

  uint8_t ack[25] = { 1, 7, 0x00, 0x6A, 0x3B, 0x2C, 0x10, 1, 0 };
  ack[9] = 0x05;   /* faltan 0 y 2 */
  ack[24] = 0x80;  /* falta 127    */
  pkt_ack_t a;
  CHECK(pkt_ack_leer(ack, 25, 1, 7, 128, &a));
  CHECK(!a.completo && a.hora == 0x6A3B2C10U && a.comando == 1);
  CHECK(pkt_ack_falta(&a, 0) && !pkt_ack_falta(&a, 1) && pkt_ack_falta(&a, 2) && pkt_ack_falta(&a, 127));
  CHECK(!pkt_ack_leer(ack, 24, 1, 7, 128, &a));          /* bitmap corto     */
  CHECK(!pkt_ack_leer(ack, 25, 2, 7, 128, &a));          /* otro bolo        */
  ack[2] = 0x80;
  CHECK(pkt_ack_leer(ack, 9, 1, 7, 128, &a) && a.completo);

  uint8_t h[6];
  CHECK(pkt_hola_armar(h, 1, 7, 99, true) == 6);
  CHECK(h[0] == 0x11 && h[3] == 0 && h[4] == 99 && h[5] == 1);
  uint8_t t[3] = { 0x21, 1, 7 };
  CHECK(pkt_teoigo_valido(t, 3, 1, 7) && !pkt_teoigo_valido(t, 3, 2, 7));

  /* Salto: la tabla es una permutacion y el registro del canal 0 es el del requisito */
  int visto[64] = { 0 };
  for (uint16_t p = 0; p < 64; p++) visto[salto_canal(0, p)]++;
  for (int i = 0; i < 64; i++) CHECK(visto[i] == 1);
  CHECK((uint32_t)(((uint64_t)salto_hz(0) << 25) / 32000000ULL) == 0x39333333U);
  CHECK(salto_hz(63) == 927800000U);
  printf("paquetes y salto: OK\n");
}

static void prueba_crc(void)
{
  const char *s = "123456789";
  CHECK(crc32_calc((const uint8_t *)s, 9, 0) == 0xCBF43926U);
  uint32_t c = crc32_calc((const uint8_t *)s, 4, 0);
  CHECK(crc32_calc((const uint8_t *)s + 4, 5, c) == 0xCBF43926U);
  CHECK(vdd_codigo(1800) == 0 && vdd_codigo(3300) == 214 && vdd_codigo(5000) == 255);
  printf("crc y vdd: OK\n");
}

/* Periodos completos, parcial, recuperado, peor caso; borrado y llenado */
static void prueba_periodos(const char *dir)
{
  static verdad_t v[6];
  char ruta[512];
  uint32_t P = T_PERIODO_S;

  almacen_vaciar();
  generar(&v[0], 100, 81600U * P,       EPOCAS_POR_PERIODO, MEDIAS_TH_POR_PERIODO, 0);
  generar(&v[1], 101, 81601U * P,       EPOCAS_POR_PERIODO, MEDIAS_TH_POR_PERIODO, 0);
  generar(&v[2], 102, 81602U * P + 740, 400, 12, 0);                       /* parcial */
  generar(&v[3], 103, 81603U * P,       EPOCAS_POR_PERIODO, MEDIAS_TH_POR_PERIODO, 1); /* peor caso */
  for (int i = 0; i < 4; i++) cargar(&v[i], i == 1);

  alm_estado_t est; almacen_estado(&est);
  printf("almacen: %u periodos, nucleo %u B, aceleracion %u B\n",
         est.periodos, est.uso_nucleo, est.uso_acel);
  CHECK(est.periodos == 4);

  /* Despues de un "reset" el almacen se revalida igual */
  almacen_init();
  almacen_estado(&est);
  CHECK(est.periodos == 4);

  /* Rafaga de un contacto: del mas nuevo al mas viejo */
  snprintf(ruta, sizeof(ruta), "%s/rafaga.txt", dir);
  FILE *f = fopen(ruta, "w"); CHECK(f);
  int nfrag = escribir_rafaga(f, 1, 42, CONT_MAX_PERIODOS);
  fclose(f);
  printf("rafaga: %d fragmentos\n", nfrag);

  snprintf(ruta, sizeof(ruta), "%s/verdad.json", dir);
  f = fopen(ruta, "w"); CHECK(f);
  fprintf(f, "[\n");
  for (int i = 3; i >= 0; i--) json_verdad(f, &v[i], i > 0);
  fprintf(f, "]\n");
  fclose(f);

  /* ACK de los 2 mas nuevos: se borran solo esos */
  CHECK(almacen_borrar(103) && almacen_borrar(102));
  CHECK(!almacen_borrar(103));
  CHECK(almacen_cantidad() == 2);
  alm_periodo_t p;
  CHECK(almacen_nuevo(0, &p) && p.n_periodo == 101);

  /* Reestampado: periodos sin hora */
  almacen_vaciar();
  periodo_abrir(946684800U + 600U, true, RST_ENCENDIDO);
  for (int i = 0; i < 30; i++) { epoca_t e = { 1, 2, 3, 30, 100 }; CHECK(periodo_epoca(&e)); }
  periodo_cierre_t c = { 7, 200, 0 }; periodo_info_t inf;
  periodo_cerrar(&c, &inf);
  almacen_reestampar(1000000);
  uint8_t meta[12]; pkt_meta_t m;
  CHECK(almacen_leer_nucleo(7, 0, meta, 12) == 12);
  pkt_meta_leer(meta, &m);
  CHECK(m.marca == 946684800U + 600U + 1000000U && m.causa_reset == RST_ENCENDIDO);
  almacen_reestampar(5);                              /* ya no tiene la bandera */
  CHECK(almacen_leer_nucleo(7, 0, meta, 12) == 12);
  pkt_meta_leer(meta, &m);
  CHECK(m.marca == 946684800U + 600U + 1000000U);
  almacen_init();                                    /* el CRC quedo bien recalculado */
  CHECK(almacen_cantidad() == 1);

  /* Llenado: periodos tipicos hasta pisar. La aceleracion se pisa antes que el nucleo. */
  almacen_vaciar();
  uint16_t n = 0;
  for (; n < 200; n++)
  {
    generar(&v[5], n, (82000U + n) * P, EPOCAS_POR_PERIODO, MEDIAS_TH_POR_PERIODO, 0);
    cargar(&v[5], 0);
  }
  almacen_estado(&est);
  int con_acel = 0;
  for (uint16_t k = 0; almacen_nuevo(k, &p); k++) if (p.largo_acel > 0U) con_acel++;
  printf("llenado con 200 periodos tipicos: quedan %u en el nucleo (%.1f dias), %d con aceleracion (%.1f dias)\n",
         est.periodos, est.periodos / 4.0, con_acel, con_acel / 4.0);
  CHECK(est.uso_nucleo <= ALM_NUCLEO_BYTES && est.uso_acel <= ALM_ACEL_BYTES);
  CHECK(almacen_nuevo(0, &p) && p.n_periodo == 199 && p.largo_acel > 0U);

  /* Un periodo sin aceleracion viaja con 0 epocas: se agrega a la rafaga de prueba */
  uint16_t viejo = 0;
  for (uint16_t k = 0; almacen_nuevo(k, &p); k++) viejo = p.n_periodo;
  CHECK(almacen_nuevo((uint16_t)(almacen_cantidad() - 1U), &p) && p.largo_acel == 0U && p.n_periodo == viejo);
  uint8_t tmp[16];
  uint32_t crc = contacto_crc_periodo(p.n_periodo, p.largo_nucleo, 0);
  CHECK(contacto_leer_periodo(p.n_periodo, p.largo_nucleo, 0, crc, 0, tmp, 12) == 12);
  pkt_meta_leer(tmp, &m);
  CHECK(m.n_epocas == 0 && m.version == FMT_VERSION && m.n_th == MEDIAS_TH_POR_PERIODO);

  /* Peor caso: cuantos fragmentos ocupa un periodo incompresible */
  almacen_vaciar();
  generar(&v[4], 1, 83000U * P, EPOCAS_POR_PERIODO, MEDIAS_TH_POR_PERIODO, 1);
  cargar(&v[4], 0);
  CHECK(almacen_nuevo(0, &p));
  uint32_t lg = contacto_largo_periodo(p.largo_nucleo, p.largo_acel);
  printf("peor caso: %u B = %u fragmentos (tope %u por contacto)\n",
         lg, (lg + PKT_FRAG_DATOS - 1) / PKT_FRAG_DATOS, CONT_MAX_FRAG);
  CHECK((lg + PKT_FRAG_DATOS - 1) / PKT_FRAG_DATOS <= CONT_MAX_FRAG);
}

int main(int argc, char **argv)
{
  const char *dir = (argc > 1) ? argv[1] : ".";
  prueba_crc();
  prueba_reloj();
  prueba_paquetes();
  prueba_motion();
  prueba_compresion(dir);
  prueba_periodos(dir);
  printf("TODO OK (C)\n");
  return 0;
}
