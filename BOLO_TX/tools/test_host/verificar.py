#!/usr/bin/env python3
"""Compara lo que codifico el firmware (test_bolo.c) con lo que decodifica Python."""
import json
import os
import random
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(AQUI))
import decodificador_bolo as dec  # noqa: E402


def falla(msg):
    print("FALLA:", msg)
    sys.exit(1)


def verificar_listas(dir_):
    n_ok = 0
    with open(os.path.join(dir_, "listas.txt")) as f:
        for linea in f:
            izq, hexa = linea.strip().split("|")
            campos = [int(x) for x in izq.split()]
            n, w, s, vals = campos[0], campos[1], campos[2], campos[3:]
            datos = bytes.fromhex(hexa)
            lb = dec.LectorBits(datos)
            out = dec.leer_lista(lb, n, w, bool(s))
            if out != vals:
                falla(f"lista n={n} w={w}: no coincide")
            if lb.byte != len(datos):
                falla(f"lista n={n} w={w}: sobran bytes ({lb.byte} de {len(datos)})")
            peor = ((w + ((n - 1 + 31) // 32) * 4 + (n - 1) * w + 7) // 8) if n else 0
            print(f"  lista n={n:4d} w={w:2d}: {len(datos):5d} B (peor caso {peor} B) OK")
            n_ok += 1
    return n_ok


def comparar(p, v):
    for k in ("n_periodo", "marca", "n_epocas", "n_th"):
        if getattr(p, k) != v[k]:
            falla(f"periodo {v['n_periodo']}: {k} = {getattr(p, k)}, esperado {v[k]}")
    for k in ("temp", "hum", "desvio", "x", "y", "z"):
        if p.crudo[k] != v[k]:
            falla(f"periodo {v['n_periodo']}: lista {k} no coincide")
    if not p.crc_ok:
        falla(f"periodo {v['n_periodo']}: CRC")
    if abs(p.vdd_v - 3.298) > 1e-9 or p.sin_ack != 3 or p.causa_reset != "watchdog":
        falla(f"periodo {v['n_periodo']}: metadatos {p.vdd_v} {p.sin_ack} {p.causa_reset}")


def verificar_rafaga(dir_):
    with open(os.path.join(dir_, "verdad.json")) as f:
        verdad = json.load(f)
    frags = []
    with open(os.path.join(dir_, "rafaga.txt")) as f:
        for linea in f:
            frags.append(dec.leer_fragmento(bytes.fromhex(linea.strip())))

    # 1. Rafaga entera, en orden
    per = dec.periodos_de_rafaga(frags)
    if len(per) != len(verdad):
        falla(f"se decodificaron {len(per)} periodos, esperados {len(verdad)}")
    for p, v in zip(per, verdad):
        comparar(p, v)
        print(f"  periodo #{p.n_periodo}: {p.largo} B en {-(-p.largo // dec.FRAG_DATOS)} fragmentos, "
              f"{p.n_epocas} epocas, {p.n_th} medias, CRC OK")

    # 2. Desordenada y con reenvios repetidos
    mezcla = frags + random.Random(1).sample(frags, 10)
    random.Random(2).shuffle(mezcla)
    per2 = dec.periodos_de_rafaga(mezcla)
    if [p.n_periodo for p in per2] != [v["n_periodo"] for v in verdad]:
        falla("con fragmentos desordenados y repetidos")
    print("  desordenada y con duplicados: OK")

    # 3. Falta un fragmento del segundo periodo: ese se pierde, los demas salen
    largo0 = -(-per[0].largo // dec.FRAG_DATOS)
    sin_uno = [f for f in frags if f.indice != largo0 + 3]
    per3 = dec.periodos_de_rafaga(sin_uno)
    esperados = [v["n_periodo"] for i, v in enumerate(verdad) if i != 1]
    if [p.n_periodo for p in per3] != esperados:
        falla(f"con un hueco: {[p.n_periodo for p in per3]} (esperados {esperados})")
    for p in per3:
        comparar(p, next(v for v in verdad if v["n_periodo"] == p.n_periodo))
    print("  con un fragmento perdido: se resincroniza y salen los otros 3: OK")

    # 4. Un byte corrupto: el CRC lo detecta
    malo = list(frags)
    f0 = malo[1]
    d = bytearray(f0.datos); d[10] ^= 0x01
    malo[1] = dec.Fragmento(f0.nodo, f0.contacto, f0.indice, f0.ultimo, bytes(d))
    per4 = dec.periodos_de_rafaga(malo)
    if verdad[0]["n_periodo"] in [p.n_periodo for p in per4]:
        falla("un periodo corrupto paso el CRC")
    print("  con un byte corrupto: el CRC lo descarta: OK")

    # 5. Unidades fisicas
    p = per[-1]
    if p.temp_c[5] is not None or p.hum_pct[5] is not None:
        falla("sin dato de temperatura")
    t_ok = [t for t in p.temp_c if t is not None]
    print(f"  ejemplo #{p.n_periodo}: T {min(t_ok):.2f}..{max(t_ok):.2f} C, "
          f"desvio {min(x for x in p.desvio_mg if x):.2f}..{max(x for x in p.desvio_mg if x):.2f} mg, "
          f"epocas invalidas {sum(1 for x in p.desvio_mg if x is None)}")


def main():
    dir_ = sys.argv[1] if len(sys.argv) > 1 else "."
    print("Listas comprimidas:")
    verificar_listas(dir_)
    print("Rafaga de un contacto:")
    verificar_rafaga(dir_)
    print("TODO OK (Python)")


if __name__ == "__main__":
    main()
