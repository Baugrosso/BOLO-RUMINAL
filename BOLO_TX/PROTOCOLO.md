# Protocolo del bolo — formato de datos y de aire (v2)

Referencia para la gateway (ESP32 + Wio-SX1262) y el decodificador de la PC. Implementa los requisitos de firmware del 2026-11-14 (T5–T12). Lo marcado como **[definido en firmware]** son huecos del requisito que hubo que cerrar para poder programar. Están en un solo lugar del código y se pueden cambiar.

Convenciones: todo **big-endian**. Los campos de bits van en el orden de la lista: el primero ocupa los bits más altos.

## 1. Radio

| | |
|---|---|
| Modulación | LoRa SF9, 125 kHz, CR 4/5, preámbulo 8, encabezado explícito, CRC |
| Payload máximo | 66 B (~390 ms en el aire) |
| Potencia | +22 dBm fijo (bolo y gateway) |
| Canales | 64 de 200 kHz: f = 915,2 MHz + 0,2 MHz × n (n = 0…63) |
| Canal de encuentro | n = 0 (915,2 MHz) **[definido en firmware: `SALTO_CANAL_ENCUENTRO`]** |
| Tabla de salto | `Core/Src/salto.c`, generada por `tools/generar_tabla_salto.py` (permutación de 0…63, semilla 0xB010) |
| TCXO / RF | TCXO por DIO3 a 1,8 V; DIO2 maneja el conmutador de RF; *fallback* a STDBY_XOSC durante el contacto |

## 2. Contacto

```
 bolo                                     gateway
  │ hola (canal de encuentro) ───────────► │   espera siempre en el canal de encuentro
  │ ◄─────────── te oigo (canal de encuentro)│   solo a IDs conocidos
  │        t0 = fin del te oigo + 50 ms     │
  │ frag 0  (ranura 0)  ───────────────────►│   canal = tabla[(32·contacto + pos) mod 64]
  │ frag 1  (ranura 1)  ───────────────────►│   una ranura = 400 ms
  │  …                                      │
  │ frag N-1 (ranura N-1) ─────────────────►│
  │ ◄──────────────────────── ACK (ranura N) │
  │ reenvíos (ranuras N+1 …) ──────────────►│   solo si el ACK pide faltantes
  │ ◄──────────────────────── ACK            │   hasta 2 rondas, máx. 32 reenviados
```

- `pos` cuenta las ranuras desde el primer fragmento: la ráfaga ocupa 0…N-1, el ACK la N, cada reenvío la siguiente, el ACK que sigue la siguiente, y así. Con N = 0 (nada que mandar) el ACK va en la ranura 0. **[definido en firmware]**
- Ranura *k* empieza en `t0 + 400 ms × k`. El bolo transmite al inicio de su ranura. La gateway manda el ACK al inicio de la ranura que le toca (el bolo escucha 400 ms desde ahí; si detectó el encabezado, espera a que termine).
- Ventana del te oigo: 500 ms después del fin del hola.
- Sin te oigo no se manda la ráfaga. Sin ACK no se borra nada.
- Los reenvíos van en orden creciente de índice, con el índice original.
- Un periodo se borra cuando todos sus fragmentos figuran como recibidos en algún ACK del contacto.

### hola — bolo → gateway, 6 B **[definido en firmware]**

| Byte | Campo |
|---|---|
| 0 | tipo (4 b) = 1 \| versión del protocolo (4 b) = 1 → `0x11` |
| 1 | ID del nodo |
| 2 | contacto (contador de 8 bits) |
| 3–4 | N = cantidad de fragmentos de la ráfaga (0…128) |
| 5 | banderas: bit 0 = no tengo ranura (pido hora y ranura) |

### te oigo — gateway → bolo, 3 B **[definido en firmware]**

`0x21` (tipo 2, versión 1) · ID del nodo · contacto (el mismo del hola)

### Fragmento — bolo → gateway, ≤ 66 B

| Byte | Campo |
|---|---|
| 0 | ID del nodo |
| 1 | contacto |
| 2–3 | índice (10 b) \| último (1 b) \| reserva (5 b, en 0) |
| 4… | hasta 62 B de datos |

*último* = 1 en el último fragmento de cada tanda (ráfaga o ronda de reenvío) **[definido en firmware]**. Cada periodo arranca en un fragmento nuevo. El último fragmento de un periodo puede ser más corto que 62 B (no se rellena por aire).

### ACK — gateway → bolo, 9 B + bitmap

| Byte | Campo |
|---|---|
| 0 | ID del nodo |
| 1 | contacto |
| 2 | estado: bit 7 = 1 → llegó todo |
| 3–6 | hora de la gateway, UNIX [s] |
| 7 | comando: 0 nada · 1 ranura · 2 potencia |
| 8 | argumento: ranura 0…179 · potencia en dBm (int8, −9…+22) |
| 9… | solo si estado bit 7 = 0: bitmap de faltantes, ⌈N/8⌉ bytes. Fragmento *i* → byte *i*/8, bit *i* % 8 (contando desde el menos significativo); 1 = falta |

## 3. Ranuras y reintentos

- Ranuras de 2 min dentro de cada bloque de 6 h (los bloques arrancan en UNIX múltiplo de 21600, o sea 00/06/12/18 h UTC). Ranura *r* → contacto a `bloque + 120 s × r` + espera al azar de 0–20 s.
- Hasta 3 intentos por ciclo, cada 4 min: `r`, `r+2`, `r+4`. Si fallan los 3 se suma 1 a "contactos sin ACK".
- Sin ranura (arranque): primer contacto al azar a 1–5 min, reintentos cada 1–3 min hasta que un ACK traiga comando 1.
- Con cada ACK el bolo ajusta su hora a la de la gateway. Un ajuste de más de 60 s (o el primero) cierra el periodo en curso como parcial.
- `BOLO_MODO_BANCO` (en `bolo_config.h`) usa periodos de 30 min y ranuras de 10 s. La gateway tiene que usar los mismos tiempos.

## 4. Periodo

Un periodo es todo lo que el bolo junta entre contactos: **6 h** (1080 épocas de 20 s y 36 medias de 10 min). En el aire:

```
metadatos (12 B) | temperatura | humedad | desvío | X | Y | Z | CRC-32 (4 B)
```

El CRC-32 es el estándar de zlib (`zlib.crc32` sobre metadatos + listas) y va en big-endian.

### Metadatos (12 B)

| Bits | Campo |
|---|---|
| 4 | versión del formato = 2 |
| 12 | cantidad de épocas (0 si la aceleración de ese periodo se pisó por falta de memoria) |
| 16 | número de periodo |
| 32 | marca de tiempo: UNIX del inicio del periodo |
| 8 | cantidad de medias de temperatura/humedad |
| 8 | VDD = 1,80 V + código × 7 mV |
| 4 \| 4 | causa del reset (0 ninguno, 1 encendido, 2 botón, 3 software, 4 watchdog, 5 HardFault, 15 desconocida) \| contactos sin ACK (saturado en 15) |
| 4 \| 4 | fallas de sensor en el periodo: acelerómetro \| temperatura (cada una saturada en 15) **[definido en firmware]** |

### Grilla

- Época *k*: `[marca + 20·k, +20 s)`. Media *j*: `[marca + 600·j, +600 s)`.
- Un periodo completo arranca en un múltiplo de 6 h. Uno **parcial** (arranque, reset, salto de hora) arranca en una época cualquiera y trae las cantidades que tenga.

### Listas

| Lista | Valores | Bits | Unidad | Sin dato |
|---|---|---|---|---|
| temperatura | cant. de medias | 16 con signo | 0,01 °C | 0x8000 |
| humedad | cant. de medias | 8 | 0,5 %RH | 0xFF |
| desvío de la magnitud | cant. de épocas | 8 | código log | 0xFF = época inválida (< 50 de 100 muestras) |
| X, Y, Z (medias) | solo épocas válidas | 12 con signo | 1 mg | — |

Desvío: σ = 0,1 mg × R^código, con R = 20000^(1/254) ≈ 1,0398 (pasos de ~4 %). El código 0 representa 0,1 mg o menos y el 254, 2 g o más.

### Compresión de una lista (sin pérdida)

Flujo de bits, el más significativo primero:

1. Primer valor entero, en el ancho de la variable (complemento a 2 si tiene signo).
2. Los n−1 restantes en bloques de 32 (el último puede ser más corto). Por cada valor, `d = v[i] − v[i−1]` y `u = zigzag(d) = (d << 1) ^ (d >> 31)`. Cada bloque lleva una etiqueta de 4 bits:
   - **0–13**: Rice con k = etiqueta: por cada `u`, `u >> k` unos, un cero y los k bits bajos de `u`.
   - **14**: sin comprimir: cada **valor** (no la diferencia), entero.
   - **15**: todas las diferencias son 0 (no hay más bits).
3. Relleno con ceros hasta el byte: la lista siguiente arranca en un byte nuevo.

El bolo elige, al cerrar cada bloque, la opción que ocupa menos. El peor caso es "sin comprimir" más 4 bits por bloque: un periodo incompresible ocupa 6134 B = 99 fragmentos.

## 5. Memoria del bolo

- **Núcleo, 8 KB** (metadatos + temperatura + humedad) y **aceleración, 40 KB**, en RAM `.noinit` (se conservan en Stop 2 y a través de un reset por watchdog; cada registro lleva su CRC).
- Se borra solo con ACK y por periodo completo. Si una zona se llena, se pisa lo más viejo de esa zona. Si se pisa la aceleración, el periodo viaja igual con 0 épocas.
- El periodo en armado también está en `.noinit`, con una foto con CRC cada 10 min. Después de un reset se recupera desde la última foto y se cierra como parcial.
- Se manda del periodo más nuevo al más viejo: hasta 4 periodos y 128 fragmentos por contacto.

## 6. Notas para la PC

- Duplicados: se descartan por **ID + número de periodo**. Ojo: el número de periodo vive en un registro de backup del RTC y vuelve a 0 con un **corte de alimentación** (reset por encendido). La PC debería reiniciar su ventana de duplicados de ese bolo cuando llega un periodo con causa de reset 1, o usar ID + número + marca de tiempo.
- Si un fragmento no llega, ese periodo no se puede decodificar en ese contacto; el bolo lo vuelve a mandar en el siguiente. El decodificador resincroniza en el siguiente periodo usando el CRC.
- Los minutos después de cada contacto la temperatura puede estar levantada por el calor del SX1262 (~231 mW a +22 dBm).

## 7. Herramientas

| Archivo | Para qué |
|---|---|
| `tools/decodificador_bolo.py` | Fragmentos (hex, uno por línea) → periodos → CSV (`--csv prefijo`) |
| `tools/generar_tabla_salto.py` | Reproduce la tabla de salto |
| `tools/test_host/correr.sh` | Compila la lógica del firmware en la PC y la verifica contra el decodificador |
