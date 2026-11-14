#!/usr/bin/env python3
"""Genera la tabla de salto pseudoaleatoria de 64 canales (T10).

La tabla tiene que ser IDENTICA en el bolo (salto.c) y en la gateway. Es una
permutacion de 0..63 sacada con una semilla fija, con la condicion de que dos
entradas seguidas (tambien la ultima con la primera) no sean canales vecinos.
Correr este script reproduce exactamente la tabla de salto.c.
"""
import random

SEMILLA = 0xB010
N = 64


def generar():
    rng = random.Random(SEMILLA)
    while True:
        t = list(range(N))
        rng.shuffle(t)
        if all(abs(t[i] - t[(i + 1) % N]) > 1 for i in range(N)):
            return t


if __name__ == "__main__":
    t = generar()
    for i in range(0, N, 16):
        print("  " + ", ".join(f"{v:2d}" for v in t[i:i + 16]) + ",")
