# Bolo-ruminal — prototipo 

Prototipo de banco de un BOLO RUMINAL: un dispositivo que va dentro del rumen de la vaca, mide MOVIMIENTO y TERMPERATURA y manda los datos por LORA (915MHZ) a un gateway.

Este repo es el **prototipo de aprendizaje**, armado con placas de desarrollo. Sirve para validar los sensores, la lógica de detección, el enlace de radio y las técnicas de ahorro de batería. El código está separado en capas para poder reutilizarlo en el bolo final (STM32WLE5, radio integrada).

> Proyecto académico — Ingeniería Electrónica, Universidad Católica de Córdoba (UCC).

---

## Sistema

```
 ┌──────────── BOLO_TX (el bolo) ────────────┐          ┌──────── BOLO_RX (gateway) ────────┐
 │ NUCLEO-L432KC                             │  LoRa    │ NUCLEO-L432KC                      │
 │  ├─ MPU-6050 (I2C)  → movimiento          │ 915 MHz  │  └─ Wio-SX1262 (SPI)               │
 │  ├─ SHT31-D  (I2C)  → temperatura/humedad │ ───────► │        │                           │
 │  └─ Wio-SX1262 (SPI) → radio              │  18 B    │        └─► USB/UART → PC (PuTTY)   │
 └───────────────────────────────────────────┘          └────────────────────────────────────┘
```

| Proyecto | Qué hace |
|---|---|
| `BOLO_TX/` | Ciclo de **reposo (STOP2) → sensado (5 s a 25 Hz) → análisis → transmisión**. Clasifica el movimiento, detecta eventos térmicos y manda un paquete binario. |
| `BOLO_RX/` | Gateway: recibe en continuo y muestra cada paquete decodificado con RSSI, SNR, paquetes perdidos y estadísticas. |

## Hardware

| Componente | Uso |
|---|---|
| NUCLEO-L432KC (×2) | Microcontrolador STM32L432KC (Cortex-M4, bajo consumo) |
| GY-521 / MPU-6050 | Acelerómetro (el giróscopo está apagado) |
| SHT31-D | Temperatura (±0,2 °C) y humedad interna (control del sellado) |
| Wio-SX1262 (862–930 MHz) | Radio LoRa, con TCXO controlado por DIO3 |

### Conexiones (iguales en TX y RX para la radio)

| Señal | Pin STM32 | Pin Nucleo |
|---|---|---|
| SPI1 SCK | PA1 | A1 |
| SPI1 MISO | PB4 | D12 |
| SPI1 MOSI | PB5 | D11 |
| LoRa NSS | PA4 | A3 |
| LoRa RST | PA7 | A6 |
| LoRa BUSY | PA0 | A0 |
| LoRa DIO1 | PA3 | A2 |
| LoRa RF_SW | PA11 | D10 |
| I2C1 SCL (solo TX) | PB6 | D5 |
| I2C1 SDA (solo TX) | PB7 | D4 |
| UART2 → PC (ST-LINK VCP) | PA2 / PA15 | — |
| LED LD3 | PB3 | D13 |

> **Nota:** en la Nucleo-32 los puentes SB16/SB18 unen PA5↔PB6 y PA6↔PB7. Por eso el SPI va en PA1/PB4/PB5 y PA5/PA6 quedan sin usar.

## Arquitectura del firmware (BOLO_TX)

```
app_bolo.c            ← orquesta el ciclo; no conoce registros ni arma textos
 ├─ motion.c          ← desvío por eje y de la magnitud → REPOSO / SUAVE / FUERTE
 ├─ thermal.c         ← línea de base adaptativa (EMA) → BAJADA / SUBA con histéresis
 ├─ bolo_packet.c     ← paquete binario v1 (18 B, little-endian, byte a byte)
 ├─ bolo_power.c      ← STOP2 + RTC, SLEEP entre muestras, HAL_Delay dormido
 ├─ report.c          ← todo lo que se imprime por UART
 ├─ sensor_acc.h / sensor_temp.h / radio.c   ← fachadas (la app no nombra chips)
 └─ drv_mpu6050.c / drv_sht31.c / drv_sx1262.c ← drivers propios
bolo_config.h         ← TODOS los parámetros ajustables (tiempos, umbrales, modo)
```

- **Driver propio del SX1262**, sin librerías de terceros: reset, TCXO, calibración, modulación, TX/RX y sleep con *warm start*.
- `radio.c` y `drv_sx1262.c` son **idénticos** en TX y RX.

### Ciclo del bolo

1. **Sensado (5 s):** 125 muestras del acelerómetro a 25 Hz, con el núcleo en SLEEP entre muestras. Al final de la ventana se mide la temperatura.
2. **Análisis:**
   - **movimiento:** 3 niveles, con dos medidas;
   - **eventos térmicos:** caída de temperatura (posible toma de agua) o subida, con resumen al terminar cada evento.
3. **Transmisión LoRa:** la radio se despierta del sleep, manda 18 B (~57 ms en el aire) y vuelve a dormir.
4. **Reposo (10 s):** STOP2, y lo despierta el RTC (LSE).

### Robustez

- **Watchdog (IWDG):** ~25 s.
- **Tope de tiempo por ventana**, y cuenta de fallas consecutivas.
- **Recuperación del bus I2C:** 9 pulsos de reloj + STOP.
- **HardFault:** deja una marca en un registro de backup del RTC y reinicia.
- **Causa de reinicio** informada en cada arranque y en el paquete.
- **Radio:** si no despierta bien del sleep, se reconfigura completa y se reintenta.

## Radio LoRa

| Parámetro | Valor |
|---|---|
| Frecuencia | 915 MHz |
| SF / BW / CR | SF7 / 125 kHz / 4/5 |
| Preámbulo | 8 símbolos, encabezado explícito, CRC activado |
| Potencia | 0 dBm (banco) |
| Topología | Punto a punto (no LoRaWAN) |

### Paquete de datos v1 (18 bytes, little-endian)

| Byte | Campo |
|---|---|
| 0 | versión (bits 7-4) \| tipo (bits 3-0) |
| 1 | ID de nodo |
| 2-3 | número de secuencia (u16) |
| 4 | movimiento (bits 1-0) \| estado térmico (bits 4-2) |
| 5 | flags: temp_ok, ventana cortada, bus recuperado, fin de evento, causa de reinicio (3 bits), evento de suba |
| 6-7 | desvío de ejes (u16, 0,1 mg) |
| 8-9 | desvío de la magnitud (u16, 0,1 mg) |
| 10-11 | temperatura (i16, 0,01 °C) |
| 12-13 | delta contra la base (i16, 0,01 °C) |
| 14 | humedad (u8, %) |
| 15 | duración del último evento (ciclos) |
| 16-17 | extremo del último evento (i16, 0,01 °C) |

## Ahorro de energía

| Fase | Cambio | Estado |
|---|---|---|
| 1 | Reloj de 32 MHz (PLL) a **MSI 4 MHz**, Voltage Scale 2, RTC con **LSE** | ✅ |
| 2 | Reposo en **STOP2**, despertado por el RTC | ✅ |
| 3 | SX1262 en **sleep (warm start)** entre transmisiones | ✅ |
| 4 | **SLEEP** entre muestras y en todos los `HAL_Delay` | ✅ |

- **Prototipo:** 2× AA directo a 3V3.
- **Bolo final:** Li-SOCl₂ ER26500 + LDO MCP1700-3302.
