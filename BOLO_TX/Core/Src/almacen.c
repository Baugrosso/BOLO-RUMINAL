/**
 * @file    almacen.c
 * @brief   Dos zonas lineales de registros con compactacion (ver almacen.h).
 *
 *  Cada zona es un arreglo donde los registros se agregan al final. Borrar solo
 *  marca el registro; el lugar se recupera compactando (se corren los registros
 *  vivos hacia el principio) cuando hace falta espacio. Es mas simple que un anillo
 *  y, como los periodos se borran del mas nuevo al mas viejo, evita los huecos.
 */
#include "almacen.h"
#include "bolo_config.h"
#include "bolo_util.h"
#include <string.h>

#ifdef BOLO_HOST
  #define NOINIT
#else
  #define NOINIT  __attribute__((section(".noinit")))
#endif

#define CAB_LARGO       12U
#define MARCA_NUC       0xB01CU
#define MARCA_ACEL      0xB0ACU
#define BAND_BORRADO    0x01U
#define BAND_SIN_HORA   0x02U

/* Offsets dentro del nucleo: la marca de tiempo esta en los bytes 4-7 de los
 * metadatos (big-endian), que son los primeros datos del registro */
#define META_MARCA_OFS  4U

typedef struct
{
  uint8_t  *mem;
  uint32_t  cap;
  uint32_t  uso;
  uint16_t  marca;
  uint16_t  pisados;
} zona_t;

static uint8_t s_mem_nuc[ALM_NUCLEO_BYTES] NOINIT __attribute__((aligned(4)));
static uint8_t s_mem_acel[ALM_ACEL_BYTES]  NOINIT __attribute__((aligned(4)));

static zona_t s_nuc  = { s_mem_nuc,  ALM_NUCLEO_BYTES, 0U, MARCA_NUC,  0U };
static zona_t s_acel = { s_mem_acel, ALM_ACEL_BYTES,   0U, MARCA_ACEL, 0U };

/* ===================== Cabecera ===================== */
static uint16_t rd16(const uint8_t *b) { return (uint16_t)(((uint16_t)b[0] << 8) | b[1]); }
static uint32_t rd32(const uint8_t *b)
{
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
}
static void wr16(uint8_t *b, uint16_t v) { b[0] = (uint8_t)(v >> 8); b[1] = (uint8_t)v; }
static void wr32(uint8_t *b, uint32_t v)
{
  b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16); b[2] = (uint8_t)(v >> 8); b[3] = (uint8_t)v;
}

static uint32_t tam_registro(uint16_t largo)
{
  return (CAB_LARGO + (uint32_t)largo + 3U) & ~3UL;
}

/* CRC de un registro ya escrito: n. periodo + largo + datos */
static uint32_t crc_registro(const uint8_t *r)
{
  uint32_t crc = crc32_calc(&r[2], 4U, 0U);
  return crc32_calc(&r[CAB_LARGO], rd16(&r[4]), crc);
}

/* ¿Hay un registro valido en 'pos'? */
static bool registro_ok(const zona_t *z, uint32_t pos)
{
  if (pos + CAB_LARGO > z->cap) return false;
  const uint8_t *r = &z->mem[pos];
  if (rd16(&r[0]) != z->marca) return false;
  uint16_t largo = rd16(&r[4]);
  if (pos + tam_registro(largo) > z->cap) return false;
  return crc_registro(r) == rd32(&r[8]);
}

/* ===================== Zona ===================== */
static void zona_validar(zona_t *z)
{
  uint32_t pos = 0U;
  while (registro_ok(z, pos)) pos += tam_registro(rd16(&z->mem[pos + 4U]));
  z->uso = pos;
}

/* Busca el registro vivo de un periodo; devuelve su posicion o -1 */
static int32_t zona_buscar(const zona_t *z, uint16_t n_periodo)
{
  for (uint32_t pos = 0U; pos < z->uso; pos += tam_registro(rd16(&z->mem[pos + 4U])))
  {
    const uint8_t *r = &z->mem[pos];
    if ((r[6] & BAND_BORRADO) == 0U && rd16(&r[2]) == n_periodo) return (int32_t)pos;
  }
  return -1;
}

static void zona_compactar(zona_t *z)
{
  uint32_t w = 0U;
  uint32_t r = 0U;

  while (r < z->uso)
  {
    uint32_t t = tam_registro(rd16(&z->mem[r + 4U]));
    if ((z->mem[r + 6U] & BAND_BORRADO) == 0U)
    {
      if (w != r) memmove(&z->mem[w], &z->mem[r], t);
      w += t;
    }
    r += t;
  }
  z->uso = w;
}

/* Marca como borrado el registro vivo mas viejo; devuelve su n. de periodo o -1 */
static int32_t zona_pisar_viejo(zona_t *z)
{
  for (uint32_t pos = 0U; pos < z->uso; pos += tam_registro(rd16(&z->mem[pos + 4U])))
  {
    uint8_t *r = &z->mem[pos];
    if ((r[6] & BAND_BORRADO) == 0U)
    {
      r[6] |= BAND_BORRADO;
      z->pisados++;
      return rd16(&r[2]);
    }
  }
  return -1;
}

static void zona_borrar(zona_t *z, uint16_t n_periodo)
{
  int32_t pos = zona_buscar(z, n_periodo);
  if (pos >= 0) z->mem[(uint32_t)pos + 6U] |= BAND_BORRADO;
}

/* Hace lugar para 'tam' bytes. 'otra' (puede ser NULL) es la zona hermana: si se
 * pisa un nucleo, su aceleracion ya no sirve y se borra tambien. */
static bool zona_lugar(zona_t *z, uint32_t tam, zona_t *otra)
{
  if (tam > z->cap) return false;
  if (z->uso + tam <= z->cap) return true;

  zona_compactar(z);
  while (z->uso + tam > z->cap)
  {
    int32_t n = zona_pisar_viejo(z);
    if (n < 0) return false;
    if (otra != NULL) zona_borrar(otra, (uint16_t)n);
    zona_compactar(z);
  }
  return true;
}

static void zona_agregar(zona_t *z, uint16_t n_periodo, uint8_t banderas,
                         const alm_parte_t *partes, uint8_t n_partes, uint16_t largo)
{
  uint8_t *r = &z->mem[z->uso];

  wr16(&r[0], z->marca);
  wr16(&r[2], n_periodo);
  wr16(&r[4], largo);
  r[6] = banderas;
  r[7] = 0U;

  uint32_t pos = CAB_LARGO;
  for (uint8_t i = 0; i < n_partes; i++)
  {
    if (partes[i].n > 0U) memcpy(&r[pos], partes[i].p, partes[i].n);
    pos += partes[i].n;
  }
  while ((pos & 3U) != 0U) r[pos++] = 0U;           /* relleno de alineacion */

  wr32(&r[8], crc_registro(r));
  z->uso += tam_registro(largo);
}

static uint16_t sumar_largos(const alm_parte_t *p, uint8_t n)
{
  uint32_t t = 0U;
  for (uint8_t i = 0; i < n; i++) t += p[i].n;
  return (t > 0xFFFFU) ? 0xFFFFU : (uint16_t)t;
}

static uint16_t zona_leer(const zona_t *z, uint16_t n_periodo, uint16_t offset,
                          uint8_t *dst, uint16_t n)
{
  int32_t pos = zona_buscar(z, n_periodo);
  if (pos < 0) return 0U;
  const uint8_t *r = &z->mem[(uint32_t)pos];
  uint16_t largo = rd16(&r[4]);
  if (offset >= largo) return 0U;
  if (n > largo - offset) n = (uint16_t)(largo - offset);
  memcpy(dst, &r[CAB_LARGO + offset], n);
  return n;
}

/* ===================== Publicas ===================== */
void almacen_init(void)
{
  zona_validar(&s_nuc);
  zona_validar(&s_acel);
}

void almacen_vaciar(void)
{
  s_nuc.uso = 0U;
  s_acel.uso = 0U;
}

bool almacen_guardar(uint16_t n_periodo, bool sin_hora,
                     const alm_parte_t *nuc, uint8_t n_nuc,
                     const alm_parte_t *acel, uint8_t n_acel)
{
  uint16_t ln = sumar_largos(nuc, n_nuc);
  uint16_t la = sumar_largos(acel, n_acel);
  uint8_t  banderas = sin_hora ? BAND_SIN_HORA : 0U;

  /* Si por algun motivo ya habia un registro con el mismo numero, se reemplaza */
  zona_borrar(&s_nuc, n_periodo);
  zona_borrar(&s_acel, n_periodo);

  if (!zona_lugar(&s_nuc, tam_registro(ln), &s_acel)) return false;
  zona_agregar(&s_nuc, n_periodo, banderas, nuc, n_nuc, ln);

  if (la > 0U && zona_lugar(&s_acel, tam_registro(la), NULL))
  {
    zona_agregar(&s_acel, n_periodo, banderas, acel, n_acel, la);
  }
  return true;
}

uint16_t almacen_cantidad(void)
{
  uint16_t n = 0U;
  for (uint32_t pos = 0U; pos < s_nuc.uso; pos += tam_registro(rd16(&s_nuc.mem[pos + 4U])))
  {
    if ((s_nuc.mem[pos + 6U] & BAND_BORRADO) == 0U) n++;
  }
  return n;
}

bool almacen_nuevo(uint16_t k, alm_periodo_t *out)
{
  uint16_t total = almacen_cantidad();
  if (k >= total) return false;

  uint16_t buscado = (uint16_t)(total - 1U - k);    /* contando desde el mas viejo */
  uint16_t i = 0U;

  for (uint32_t pos = 0U; pos < s_nuc.uso; pos += tam_registro(rd16(&s_nuc.mem[pos + 4U])))
  {
    const uint8_t *r = &s_nuc.mem[pos];
    if ((r[6] & BAND_BORRADO) != 0U) continue;
    if (i++ != buscado) continue;

    out->n_periodo    = rd16(&r[2]);
    out->largo_nucleo = rd16(&r[4]);
    out->sin_hora     = (r[6] & BAND_SIN_HORA) != 0U;
    int32_t pa = zona_buscar(&s_acel, out->n_periodo);
    out->largo_acel   = (pa >= 0) ? rd16(&s_acel.mem[(uint32_t)pa + 4U]) : 0U;
    return true;
  }
  return false;
}

uint16_t almacen_leer_nucleo(uint16_t n_periodo, uint16_t offset, uint8_t *dst, uint16_t n)
{
  return zona_leer(&s_nuc, n_periodo, offset, dst, n);
}

uint16_t almacen_leer_acel(uint16_t n_periodo, uint16_t offset, uint8_t *dst, uint16_t n)
{
  return zona_leer(&s_acel, n_periodo, offset, dst, n);
}

bool almacen_borrar(uint16_t n_periodo)
{
  bool habia = zona_buscar(&s_nuc, n_periodo) >= 0;
  zona_borrar(&s_nuc, n_periodo);
  zona_borrar(&s_acel, n_periodo);
  return habia;
}

void almacen_reestampar(int32_t delta)
{
  for (uint32_t pos = 0U; pos < s_nuc.uso; pos += tam_registro(rd16(&s_nuc.mem[pos + 4U])))
  {
    uint8_t *r = &s_nuc.mem[pos];
    if ((r[6] & (BAND_BORRADO | BAND_SIN_HORA)) != BAND_SIN_HORA) continue;
    if (rd16(&r[4]) < META_MARCA_OFS + 4U) continue;

    uint8_t *m = &r[CAB_LARGO + META_MARCA_OFS];
    wr32(m, (uint32_t)((int32_t)rd32(m) + delta));
    r[6] &= (uint8_t)~BAND_SIN_HORA;
    wr32(&r[8], crc_registro(r));

    int32_t pa = zona_buscar(&s_acel, rd16(&r[2]));
    if (pa >= 0) s_acel.mem[(uint32_t)pa + 6U] &= (uint8_t)~BAND_SIN_HORA;
  }
}

void almacen_estado(alm_estado_t *e)
{
  e->uso_nucleo     = s_nuc.uso;
  e->uso_acel       = s_acel.uso;
  e->periodos       = almacen_cantidad();
  e->pisados_nucleo = s_nuc.pisados;
  e->pisados_acel   = s_acel.pisados;
}
