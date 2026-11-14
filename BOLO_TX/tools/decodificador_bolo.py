#!/usr/bin/env python3
"""Decodificador de los datos del bolo (PC / servidor).

La gateway (ESP32) reenvia los fragmentos tal cual los recibe. Este modulo:
  1. junta los fragmentos de un contacto por indice (descarta duplicados),
  2. corta la rafaga en periodos (cada periodo arranca en un fragmento nuevo y
     termina en su CRC-32; si falta un pedazo, se resincroniza probando el CRC),
  3. descomprime cada periodo y devuelve los valores en unidades fisicas.

Formato: ver BOLO_TX/PROTOCOLO.md (es la referencia; este archivo la implementa).

Uso por linea de comandos:
  python3 decodificador_bolo.py fragmentos.txt        # un fragmento por linea, en hex
  python3 decodificador_bolo.py --csv salida fragmentos.txt
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import zlib
from dataclasses import dataclass, field

# ---------------------------------------------------------------- constantes
FRAG_CAB = 4
FRAG_DATOS = 62
META_LARGO = 12
CRC_LARGO = 4
BLOQUE = 32
ETQ_SIN_COMPRIMIR = 14
ETQ_CEROS = 15

T_EPOCA_S = 20
T_MEDIA_TH_S = 600

TEMP_SIN_DATO = -32768        # 0x8000
HUM_SIN_DATO = 0xFF
DESVIO_INVALIDO = 0xFF
DESVIO_R = 20000.0 ** (1.0 / 254.0)

CAUSAS_RESET = {0: "ninguno", 1: "encendido", 2: "boton", 3: "software",
                4: "watchdog", 5: "hardfault", 15: "desconocida"}


class ErrorFormato(Exception):
    pass


# ---------------------------------------------------------------- bits
class LectorBits:
    """Lee un flujo de bits, el mas significativo primero."""

    def __init__(self, datos: bytes, inicio: int = 0):
        self.d = datos
        self.pos = inicio * 8           # en bits

    def bit(self) -> int:
        if self.pos >= len(self.d) * 8:
            raise ErrorFormato("se termino el periodo antes de tiempo")
        b = (self.d[self.pos >> 3] >> (7 - (self.pos & 7))) & 1
        self.pos += 1
        return b

    def bits(self, n: int) -> int:
        v = 0
        for _ in range(n):
            v = (v << 1) | self.bit()
        return v

    def alinear(self):
        self.pos = (self.pos + 7) & ~7

    @property
    def byte(self) -> int:
        return self.pos >> 3


def _con_signo(v: int, ancho: int) -> int:
    return v - (1 << ancho) if v & (1 << (ancho - 1)) else v


def _des_zigzag(u: int) -> int:
    return (u >> 1) ^ -(u & 1)


def leer_lista(lb: LectorBits, n: int, ancho: int, signo: bool) -> list[int]:
    """Descomprime una lista de n valores (compresion.h) y alinea al byte."""
    if n == 0:
        return []
    v = lb.bits(ancho)
    if signo:
        v = _con_signo(v, ancho)
    vals = [v]
    resto = n - 1
    while resto > 0:
        cnt = min(BLOQUE, resto)
        etq = lb.bits(4)
        if etq == ETQ_CEROS:
            vals.extend([vals[-1]] * cnt)
        elif etq == ETQ_SIN_COMPRIMIR:
            for _ in range(cnt):
                x = lb.bits(ancho)
                vals.append(_con_signo(x, ancho) if signo else x)
        else:
            k = etq
            for _ in range(cnt):
                q = 0
                while lb.bit() == 1:
                    q += 1
                u = (q << k) | (lb.bits(k) if k else 0)
                vals.append(vals[-1] + _des_zigzag(u))
        resto -= cnt
    lb.alinear()
    return vals


# ---------------------------------------------------------------- periodo
@dataclass
class Periodo:
    version: int
    n_periodo: int
    marca: int                      # UNIX [s], inicio del periodo
    n_epocas: int
    n_th: int
    vdd_v: float
    causa_reset: str
    sin_ack: int
    fallas_acc: int
    fallas_temp: int
    temp_c: list = field(default_factory=list)      # None = sin dato
    hum_pct: list = field(default_factory=list)
    desvio_mg: list = field(default_factory=list)   # None = epoca invalida
    x_mg: list = field(default_factory=list)        # None en las epocas invalidas
    y_mg: list = field(default_factory=list)
    z_mg: list = field(default_factory=list)
    crc_ok: bool = True
    largo: int = 0                  # bytes que ocupa en el aire (con CRC)
    crudo: dict = field(default_factory=dict)       # listas enteras sin convertir

    def tiempo_epoca(self, k: int) -> int:
        return self.marca + T_EPOCA_S * k

    def tiempo_th(self, j: int) -> int:
        return self.marca + T_MEDIA_TH_S * j


def desvio_mg(cod: int):
    if cod == DESVIO_INVALIDO:
        return None
    return 0.1 * DESVIO_R ** cod


def leer_meta(b: bytes) -> dict:
    if len(b) < META_LARGO:
        raise ErrorFormato("metadatos incompletos")
    v0 = int.from_bytes(b[0:2], "big")
    return {
        "version": v0 >> 12,
        "n_epocas": v0 & 0x0FFF,
        "n_periodo": int.from_bytes(b[2:4], "big"),
        "marca": int.from_bytes(b[4:8], "big"),
        "n_th": b[8],
        "vdd_cod": b[9],
        "causa_reset": b[10] >> 4,
        "sin_ack": b[10] & 0x0F,
        "fallas_acc": b[11] >> 4,
        "fallas_temp": b[11] & 0x0F,
    }


def decodificar_periodo(b: bytes, inicio: int = 0) -> Periodo:
    """Decodifica un periodo que arranca en b[inicio]. Verifica el CRC-32."""
    m = leer_meta(b[inicio:inicio + META_LARGO])
    if m["version"] != 2:
        raise ErrorFormato(f"version de formato desconocida: {m['version']}")
    lb = LectorBits(b, inicio + META_LARGO)
    temp = leer_lista(lb, m["n_th"], 16, True)
    hum = leer_lista(lb, m["n_th"], 8, False)
    desv = leer_lista(lb, m["n_epocas"], 8, False)
    n_val = sum(1 for d in desv if d != DESVIO_INVALIDO)
    xs = leer_lista(lb, n_val, 12, True)
    ys = leer_lista(lb, n_val, 12, True)
    zs = leer_lista(lb, n_val, 12, True)
    fin = lb.byte
    if fin + CRC_LARGO > len(b):
        raise ErrorFormato("falta el CRC")
    crc = int.from_bytes(b[fin:fin + CRC_LARGO], "big")
    crc_ok = (zlib.crc32(b[inicio:fin]) & 0xFFFFFFFF) == crc

    p = Periodo(
        version=m["version"], n_periodo=m["n_periodo"], marca=m["marca"],
        n_epocas=m["n_epocas"], n_th=m["n_th"],
        vdd_v=round(1.80 + 0.007 * m["vdd_cod"], 3),
        causa_reset=CAUSAS_RESET.get(m["causa_reset"], str(m["causa_reset"])),
        sin_ack=m["sin_ack"], fallas_acc=m["fallas_acc"], fallas_temp=m["fallas_temp"],
        crc_ok=crc_ok, largo=fin + CRC_LARGO - inicio,
        crudo={"temp": temp, "hum": hum, "desvio": desv, "x": xs, "y": ys, "z": zs},
    )
    p.temp_c = [None if t == TEMP_SIN_DATO else t / 100.0 for t in temp]
    p.hum_pct = [None if h == HUM_SIN_DATO else h * 0.5 for h in hum]
    p.desvio_mg = [desvio_mg(d) for d in desv]
    it = iter(zip(xs, ys, zs))
    for d in desv:
        if d == DESVIO_INVALIDO:
            p.x_mg.append(None); p.y_mg.append(None); p.z_mg.append(None)
        else:
            x, y, z = next(it)
            p.x_mg.append(x); p.y_mg.append(y); p.z_mg.append(z)
    return p


# ---------------------------------------------------------------- rafaga
@dataclass
class Fragmento:
    nodo: int
    contacto: int
    indice: int
    ultimo: bool
    datos: bytes


def leer_fragmento(paquete: bytes) -> Fragmento:
    if len(paquete) < FRAG_CAB + 1:
        raise ErrorFormato("fragmento demasiado corto")
    w = int.from_bytes(paquete[2:4], "big")
    return Fragmento(nodo=paquete[0], contacto=paquete[1], indice=w >> 6,
                     ultimo=bool(w & 0x20), datos=bytes(paquete[FRAG_CAB:]))


def periodos_de_rafaga(fragmentos: list[Fragmento]) -> list[Periodo]:
    """Arma los periodos de un contacto. Los fragmentos pueden venir desordenados,
    repetidos (reenvios) o con huecos: un periodo con un hueco se pierde (el bolo
    lo vuelve a mandar en el proximo contacto) y se sigue con el siguiente."""
    por_indice = {}
    for f in fragmentos:
        por_indice.setdefault(f.indice, f)
    if not por_indice:
        return []
    n = max(por_indice) + 1
    periodos = []
    i = 0
    while i < n:
        if i not in por_indice:
            i += 1
            continue
        # Juntar fragmentos seguidos desde i y probar si arranca un periodo
        datos = b""
        j = i
        ok = None
        while j < n and j in por_indice:
            datos += por_indice[j].datos
            j += 1
            try:
                p = decodificar_periodo(datos)
            except ErrorFormato:
                continue
            if p.crc_ok and p.largo <= len(datos):
                ok = p
                break
        if ok is not None:
            periodos.append(ok)
            i += math.ceil(ok.largo / FRAG_DATOS)
        else:
            i += 1          # no arranca un periodo aca: resincronizar en el siguiente
    return periodos


# ---------------------------------------------------------------- salida
def filas_epocas(p: Periodo, nodo: int):
    for k in range(p.n_epocas):
        yield {"nodo": nodo, "periodo": p.n_periodo, "unix": p.tiempo_epoca(k),
               "x_mg": p.x_mg[k], "y_mg": p.y_mg[k], "z_mg": p.z_mg[k],
               "desvio_mg": None if p.desvio_mg[k] is None else round(p.desvio_mg[k], 3)}


def filas_th(p: Periodo, nodo: int):
    for j in range(p.n_th):
        yield {"nodo": nodo, "periodo": p.n_periodo, "unix": p.tiempo_th(j),
               "temp_c": p.temp_c[j], "hum_pct": p.hum_pct[j]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("archivo", help="fragmentos en hex, uno por linea (cabecera incluida)")
    ap.add_argument("--csv", metavar="PREFIJO", help="escribir PREFIJO_epocas.csv y PREFIJO_th.csv")
    a = ap.parse_args(argv)

    frags = []
    with open(a.archivo) as fh:
        for linea in fh:
            linea = linea.strip().replace(" ", "")
            if linea and not linea.startswith("#"):
                frags.append(leer_fragmento(bytes.fromhex(linea)))
    if not frags:
        print("sin fragmentos", file=sys.stderr)
        return 1
    nodo = frags[0].nodo
    periodos = periodos_de_rafaga(frags)
    for p in periodos:
        print(f"nodo {nodo} periodo #{p.n_periodo} desde {p.marca}: {p.n_epocas} epocas, "
              f"{p.n_th} medias T/HR, VDD {p.vdd_v} V, reset {p.causa_reset}, "
              f"sin ACK {p.sin_ack}, CRC {'OK' if p.crc_ok else 'MAL'}")
    if a.csv:
        with open(a.csv + "_epocas.csv", "w", newline="") as fe, open(a.csv + "_th.csv", "w", newline="") as ft:
            we = csv.DictWriter(fe, fieldnames=["nodo", "periodo", "unix", "x_mg", "y_mg", "z_mg", "desvio_mg"])
            wt = csv.DictWriter(ft, fieldnames=["nodo", "periodo", "unix", "temp_c", "hum_pct"])
            we.writeheader(); wt.writeheader()
            for p in periodos:
                we.writerows(filas_epocas(p, nodo))
                wt.writerows(filas_th(p, nodo))
    return 0


if __name__ == "__main__":
    sys.exit(main())
