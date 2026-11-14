#!/bin/bash
# Compila la logica del firmware en la PC y la verifica contra el decodificador Python.
#   ./tools/test_host/correr.sh            (desde BOLO_TX/)
set -e
AQUI="$(cd "$(dirname "$0")" && pwd)"
RAIZ="$AQUI/../.."
SAL="${TMPDIR:-/tmp}/bolo_test"
mkdir -p "$SAL"
gcc -std=gnu11 -O2 -Wall -Wextra -DBOLO_HOST -I"$AQUI/stub" -I"$RAIZ/Core/Inc" \
  "$AQUI/test_bolo.c" \
  "$RAIZ"/Core/Src/{compresion,periodo,almacen,bolo_packet,contacto,bolo_util,motion,bolo_reloj,salto}.c \
  -lm -o "$SAL/test_bolo"
"$SAL/test_bolo" "$SAL"
python3 "$AQUI/verificar.py" "$SAL"
