/**
 * @file    compresion.c
 * @brief   Codificador: diferencias + zigzag + Rice por bloques de 32 (ver compresion.h).
 *          El decodificador es el de Python (tools/decodificador_bolo.py).
 */
#include "compresion.h"
#include <string.h>

/* ===================== Bits ===================== */
/* Escribe los 'nb' bits bajos de 'v', el mas significativo primero.
 * El buffer arranca en cero (cmp_iniciar), asi que alcanza con poner los unos. */
static void poner_bits(uint8_t *buf, uint32_t *pos, uint32_t v, uint8_t nb)
{
  for (int8_t i = (int8_t)nb - 1; i >= 0; i--)
  {
    if ((v >> (uint8_t)i) & 1U) buf[*pos >> 3] |= (uint8_t)(0x80U >> (*pos & 7U));
    (*pos)++;
  }
}

static void poner_unos(uint8_t *buf, uint32_t *pos, uint32_t cuantos)
{
  for (uint32_t i = 0; i < cuantos; i++)
  {
    buf[*pos >> 3] |= (uint8_t)(0x80U >> (*pos & 7U));
    (*pos)++;
  }
}

static uint32_t zigzag(int32_t d)
{
  return ((uint32_t)d << 1) ^ (uint32_t)(d >> 31);
}

static uint32_t mascara(uint8_t ancho)
{
  return (ancho >= 32U) ? 0xFFFFFFFFU : ((1UL << ancho) - 1U);
}

/* ===================== Bloque ===================== */
/* Elige la etiqueta mas barata para el bloque actual y lo escribe */
static bool cerrar_bloque(cmp_t *c, uint8_t *buf)
{
  uint32_t u[CMP_BLOQUE];
  uint8_t  cnt = c->en_bloque;
  bool     ceros = true;
  int32_t  ant = c->base_bloque;

  if (cnt == 0U) return true;

  for (uint8_t i = 0; i < cnt; i++)
  {
    u[i] = zigzag(c->bloque[i] - ant);
    ant  = c->bloque[i];
    if (u[i] != 0U) ceros = false;
  }

  /* Costo de cada opcion (sin contar los 4 bits de la etiqueta) */
  uint8_t  etq   = CMP_ETQ_SIN_COMPRIMIR;
  uint32_t costo = (uint32_t)cnt * c->ancho;

  if (ceros)
  {
    etq = CMP_ETQ_CEROS;
    costo = 0U;
  }
  else
  {
    for (uint8_t k = 0; k <= CMP_K_MAX; k++)
    {
      uint32_t ck = (uint32_t)cnt * (1U + k);
      for (uint8_t i = 0; i < cnt && ck < costo; i++) ck += u[i] >> k;
      if (ck < costo) { costo = ck; etq = k; }
    }
  }

  /* ¿Entra? */
  if (c->bits + 4U + costo > (uint32_t)c->cap * 8U)
  {
    c->desborde = 1U;
    return false;
  }

  poner_bits(buf, &c->bits, etq, 4U);

  if (etq == CMP_ETQ_SIN_COMPRIMIR)
  {
    uint32_t m = mascara(c->ancho);
    for (uint8_t i = 0; i < cnt; i++) poner_bits(buf, &c->bits, (uint32_t)c->bloque[i] & m, c->ancho);
  }
  else if (etq <= CMP_K_MAX)
  {
    for (uint8_t i = 0; i < cnt; i++)
    {
      poner_unos(buf, &c->bits, u[i] >> etq);       /* cociente en unario...  */
      poner_bits(buf, &c->bits, 0U, 1U);            /* ...terminado en 0      */
      if (etq > 0U) poner_bits(buf, &c->bits, u[i] & mascara(etq), etq);
    }
  }
  /* CMP_ETQ_CEROS: nada mas */

  c->base_bloque = c->bloque[cnt - 1U];
  c->en_bloque   = 0U;
  return true;
}

/* ===================== Publicas ===================== */
void cmp_iniciar(cmp_t *c, uint8_t *buf, uint16_t cap, uint8_t ancho, bool con_signo)
{
  memset(c, 0, sizeof(*c));
  c->cap       = cap;
  c->ancho     = ancho;
  c->con_signo = con_signo ? 1U : 0U;
  if (buf != NULL && cap > 0U) memset(buf, 0, cap);
}

bool cmp_agregar(cmp_t *c, uint8_t *buf, int32_t v)
{
  if (c->desborde || c->n == 0xFFFFU) return false;

  if (c->n == 0U)
  {
    /* Primer valor: entero */
    if (c->bits + c->ancho > (uint32_t)c->cap * 8U) { c->desborde = 1U; return false; }
    poner_bits(buf, &c->bits, (uint32_t)v & mascara(c->ancho), c->ancho);
    c->base_bloque = v;
  }
  else
  {
    c->bloque[c->en_bloque++] = v;
    if (c->en_bloque == CMP_BLOQUE && !cerrar_bloque(c, buf)) return false;
  }

  c->previo = v;
  c->n++;
  return true;
}

uint16_t cmp_cerrar(cmp_t *c, uint8_t *buf)
{
  (void)cerrar_bloque(c, buf);
  return (uint16_t)((c->bits + 7U) / 8U);
}
