/**
 * @file    bolo_config.h
 * @brief   Parametros de configuracion del BOLO_TX (prototipo, primera iteracion).
 *          Todo valor ajustable del sistema vive aca y solo aca.
 *
 *  Fuente de verdad: "Requisitos de firmware y comunicacion del bolo" (2026-11-14).
 *  Criterio general: el bolo NO clasifica. Manda lo mas crudo que entre y toda la
 *  interpretacion (movimiento, toma de agua) se hace en la gateway / servidor.
 */
#ifndef BOLO_CONFIG_H
#define BOLO_CONFIG_H

#include <stdint.h>

/* =========================================================
 *  1. IDENTIDAD
 * ========================================================= */
#define BOLO_NODO_ID            1U      /* 1..255. Esta iteracion: bolos 1 y 2 */

/* =========================================================
 *  2. MODO DE TIEMPOS
 * ========================================================= */
/* REAL : los tiempos del requisito (periodo de 6 h, ranuras de 2 min).
 * BANCO: los mismos mecanismos con tiempos cortos para probar en la mesa
 *        (periodo de 30 min, ranuras de 10 s). La gateway tiene que usar los
 *        MISMOS valores: si no, no se encuentran. */
#define BOLO_MODO_REAL          1
#define BOLO_MODO_BANCO         2

#define BOLO_MODO               BOLO_MODO_REAL   /* <-- se cambia SOLO esta linea */

/* =========================================================
 *  3. GRILLA DE TIEMPO (T5/T6)
 * ========================================================= */
#define T_EPOCA_S               20U     /* epoca de aceleracion (opcion B)          */
#define T_SHT_S                 60U     /* una medicion de temperatura por minuto   */
#define T_MEDIA_TH_S            600U    /* se manda la media de cada 10 min         */

#if   (BOLO_MODO == BOLO_MODO_REAL)
  #define T_PERIODO_S           21600U  /* 6 h: un periodo = lo que junta entre contactos */
  #define T_RANURA_S            120U    /* ranuras de 2 min (0-179)                 */
  #define T_ESPERA_HOLA_MAX_S   20U     /* espera al azar de 0-20 s antes del hola  */
  #define T_ARRANQUE_MIN_S      60U     /* sin ranura: primer contacto a 1-5 min    */
  #define T_ARRANQUE_MAX_S      300U
  #define T_REINTENTO_MIN_S     60U     /* sin ranura: reintentos cada 1-3 min      */
  #define T_REINTENTO_MAX_S     180U
#elif (BOLO_MODO == BOLO_MODO_BANCO)
  #define T_PERIODO_S           1800U
  #define T_RANURA_S            10U
  #define T_ESPERA_HOLA_MAX_S   3U
  #define T_ARRANQUE_MIN_S      10U
  #define T_ARRANQUE_MAX_S      30U
  #define T_REINTENTO_MIN_S     10U
  #define T_REINTENTO_MAX_S     20U
#else
  #error "BOLO_MODO no valido"
#endif

/* Derivados (no tocar) */
#define EPOCAS_POR_PERIODO      (T_PERIODO_S / T_EPOCA_S)       /* REAL: 1080 */
#define MEDIAS_TH_POR_PERIODO   (T_PERIODO_S / T_MEDIA_TH_S)    /* REAL: 36   */
#define SHT_POR_MEDIA           (T_MEDIA_TH_S / T_SHT_S)        /* 10         */
#define RANURAS_POR_PERIODO     (T_PERIODO_S / T_RANURA_S)      /* REAL: 180  */

_Static_assert((T_PERIODO_S % T_MEDIA_TH_S) == 0U && (T_MEDIA_TH_S % T_SHT_S) == 0U &&
               (T_PERIODO_S % T_EPOCA_S) == 0U && (T_SHT_S % T_EPOCA_S) == 0U,
               "La grilla tiene que ser exacta (6 h / 10 min / 1 min / 20 s)");
_Static_assert(EPOCAS_POR_PERIODO <= 4095U,   "Cant. de epocas: 12 bits en los metadatos");
_Static_assert(MEDIAS_TH_POR_PERIODO <= 255U, "Cant. de temperaturas: 8 bits en los metadatos");
_Static_assert(RANURAS_POR_PERIODO <= 256U,   "La ranura viaja en 8 bits (argumento del ACK)");

/* =========================================================
 *  4. ACELEROMETRO (MPU-6050, solo acelerometro)
 * ========================================================= */
/* Low-power cycle a 5 Hz, +-2 g, giroscopo apagado. El MPU avisa cada muestra por
 * su pin INT (plan A), cableado a PB1 (D6). */
#define ACC_FS_HZ               5U
#define ACC_MUESTRAS_EPOCA      (ACC_FS_HZ * T_EPOCA_S)          /* 100 */
#define ACC_MIN_VALIDAS         50U     /* < 50 de 100 muestras -> epoca invalida */

#define BOLO_MPU_INT_PORT       GPIOB
#define BOLO_MPU_INT_PIN        GPIO_PIN_1      /* D6 */
#define BOLO_MPU_INT_IRQn       EXTI1_IRQn

/* =========================================================
 *  5. TEMPERATURA (SHT31-D)
 * ========================================================= */
#define TEMP_I2C_ADDR           0x44U   /* ADR a GND (0x45 si ADR en alto) */

/* Pines del bus I2C1, para destrabarlo a mano si un sensor deja SDA en bajo */
#define BOLO_I2C_PORT           GPIOB
#define BOLO_I2C_SCL_PIN        GPIO_PIN_6     /* D5 */
#define BOLO_I2C_SDA_PIN        GPIO_PIN_7     /* D4 */

/* =========================================================
 *  6. FORMATO DE DATOS (T6) y COMPRESION (T7)
 * ========================================================= */
#define FMT_VERSION             2U      /* version del formato del periodo (4 bits)  */
#define FMT_TEMP_SIN_DATO       ((int16_t)0x8000)
#define FMT_HUM_SIN_DATO        0xFFU
#define FMT_DESVIO_INVALIDO     0xFFU   /* epoca con < 50 muestras                   */
#define FMT_ACC_MAX_MG          2047    /* medias X/Y/Z: 12 bits con signo, 1 mg     */
#define FMT_ACC_MIN_MG          (-2048)

#define CMP_BLOQUE              32U     /* valores por bloque Rice */

/* =========================================================
 *  7. MEMORIA (T8): todo en RAM, se conserva en Stop 2
 * ========================================================= */
#define ALM_NUCLEO_BYTES        (8U * 1024U)    /* temperatura, humedad, metadatos */
#define ALM_ACEL_BYTES          (40U * 1024U)   /* aceleracion                     */

/* =========================================================
 *  8. CONTACTO (T9-T12)
 * ========================================================= */
#define CONT_MAX_PERIODOS       4U      /* del mas nuevo al mas viejo                 */
#define CONT_MAX_FRAG           128U    /* 128 fragmentos -> bitmap del ACK de 16 B   */
#define CONT_RONDAS_REENVIO     2U
#define CONT_MAX_REENVIOS       32U
#define CONT_INTENTOS           3U      /* por ciclo, en las ranuras propias          */
#define CONT_SEPARACION_RANURAS 2U      /* intentos cada 4 min = 2 ranuras            */

/* =========================================================
 *  9. RADIO
 * ========================================================= */
#define RADIO_POTENCIA_DBM      22      /* fijo durante todo el contacto (T11) */

/* =========================================================
 *  10. INDICADORES Y DEPURACION (impactan en la bateria)
 * ========================================================= */
/* 0 = mudo (consumo real) | 1 = eventos (periodos, contactos) | 2 = detalle (cada epoca y cada temperatura) */
#define BOLO_LOG                1
#define BOLO_UART_COLOR         1   /* 1 = colores en PuTTY                         */
#define BOLO_LED_CONTACTO       1   /* 1 = LD3 encendido durante cada contacto      */
#define BOLO_DIAG_I2C           1   /* 1 = escaneo del bus I2C al arrancar          */

/* 1 = el debugger sigue conectado en STOP2 (cientos de uA mas). Para MEDIR, en 0. */
#define BOLO_DBG_EN_STOP        0

/* =========================================================
 *  11. WATCHDOG (IWDG) y FALLAS
 * ========================================================= */
/* CubeMX: prescaler 256, reload 3124 -> 25 s nominal, ~23 s en el peor caso del LSI.
 * Se refresca en cada tick de 1 s del RTC y en cada fragmento. */
#define IWDG_TIMEOUT_MIN_S      23U

/* Marca que deja el HardFault_Handler en un registro de backup del RTC antes de
 * resetear (sobrevive al reset): asi el arranque sabe que fue un HardFault. */
#define BOLO_MARCA_HARDFAULT    0xBAD0FA17U

/* Un ajuste de hora mas grande que esto (en cualquier sentido) cierra el periodo en
 * curso como parcial y arranca uno nuevo en la grilla nueva. Los ajustes chicos
 * (deriva del cristal) se absorben sin cortar nada. */
#define BOLO_SALTO_HORA_MAX_S   60U

#endif /* BOLO_CONFIG_H */
