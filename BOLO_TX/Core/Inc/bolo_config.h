/**
 * @file    bolo_config.h
 * @brief   Parámetros de configuración del prototipo BOLO_TX.
 *          Todo valor ajustable del sistema vive acá y solo acá.
 */
#ifndef BOLO_CONFIG_H
#define BOLO_CONFIG_H

/* =========================================================
 *  1. MODO DE FUNCIONAMIENTO
 * ========================================================= */
#define BOLO_MODO_DEMO     1   /* Ciclos cortos: para mostrar en vivo        */
#define BOLO_MODO_PRUEBA   2   /* Ciclos de banco: desarrollo y pruebas      */

#define BOLO_MODO          BOLO_MODO_PRUEBA   /* <-- se cambia SOLO esta línea */

/* =========================================================
 *  2. VENTANAS DE TIEMPO DEL CICLO
 * ========================================================= */
#if   (BOLO_MODO == BOLO_MODO_DEMO)
  #define T_REPOSO_S        5U    /* segundos durmiendo  */
  #define T_SENSADO_S       5U    /* segundos midiendo   */
#elif (BOLO_MODO == BOLO_MODO_PRUEBA)
  #define T_REPOSO_S        10U
  #define T_SENSADO_S       5U
#else
  #error "BOLO_MODO no valido"
#endif
#define T_CICLO_S          (T_REPOSO_S + T_SENSADO_S)   /* periodo entre mediciones */

/* =========================================================
 *  3. MUESTREO DEL ACELEROMETRO
 * ========================================================= */
#define ACC_FS_HZ           25U                          /* muestras por segundo     */
#define ACC_PERIODO_MS      (1000U / ACC_FS_HZ)          /* 40 ms entre muestras     */
#define ACC_N_MUESTRAS      (ACC_FS_HZ * T_SENSADO_S)    /* 125 muestras por ventana */

/* =========================================================
 *  4. DETECCION DE MOVIMIENTO (3 niveles, 2 medidas)
 * ========================================================= */
/* Medida 1: mayor desvio de los ejes X/Y/Z -> "¿se movio?"  (rotacion o sacudida)
 * Medida 2: desvio de la magnitud |a|      -> "¿fue brusco?" (en una rotacion pura
 *                                             la magnitud se queda en 1 g)
 *   desvio_mag >= FUERTE            -> MOVIMIENTO FUERTE
 *   si no, desvio_ejes >= SUAVE     -> MOVIMIENTO SUAVE
 *   si no                           -> REPOSO
 * Valores en mili-g. SUAVE calibrado en banco (18/10/2026).
 * FUERTE: valor inicial, a calibrar con rotacion vs sacudida. */
#define MOV_UMBRAL_SUAVE_MG    5U     /* sobre el desvio de los ejes    */
#define MOV_UMBRAL_FUERTE_MG   50U    /* sobre el desvio de la magnitud */
/* =========================================================
 *  5. INDICADORES Y DEPURACION (impactan en la bateria)
 * ========================================================= */
#define BOLO_LED_SENSADO    1   /* 1 = LED encendido mientras mide (demo)    */
#define BOLO_UART_DETALLE   1   /* 1 = imprime el reporte completo por UART  */
#define BOLO_UART_COLOR     1   /* 1 = colores en PuTTY (0 si el log muestra simbolos raros) */
#define BOLO_DIAG_I2C       1   /* 1 = escaneo del bus I2C al arrancar (diagnostico)          */
/* =========================================================
 *  6. CHEQUEOS EN TIEMPO DE COMPILACION
 * ========================================================= */
/* La FIFO del MPU-6050 tiene 1024 bytes y cada muestra (X,Y,Z) ocupa 6.
 * Lo verificamos ya, porque es la estrategia de bajo consumo que viene. */
_Static_assert((ACC_N_MUESTRAS * 6U) <= 1024U,
        "La ventana de sensado no entra en la FIFO del MPU-6050");
/* =========================================================
 *  7. SENSOR DE TEMPERATURA (SHT31-D)
 * ========================================================= */
#define TEMP_I2C_ADDR       0x44U   /* ADR a GND (0x45 si ADR esta en alto) */
/* La medicion se dispara al FINAL de la ventana (despues de esta muestra), asi
 * representa lo que paso durante el sensado y no el instante en que se prende el LED.
 * Queda 1 turno (40 ms) hasta el final, y el SHT31 tarda como mucho 15 ms. */
#define TEMP_INICIO_MUESTRA (ACC_N_MUESTRAS - 2U)
_Static_assert(((ACC_N_MUESTRAS - 1U - TEMP_INICIO_MUESTRA) * ACC_PERIODO_MS) >= 15U,
        "No queda tiempo para que el SHT31 termine de medir antes del final de la ventana");
/* =========================================================
 *  8. ROBUSTEZ (que una falla de sensor no cuelgue el ciclo)
 * ========================================================= */
#define VENTANA_MAX_MS      (T_SENSADO_S * 1000U + 1000U)  /* 6 s: tope de la ventana   */
#define ACC_FALLAS_MAX      5U      /* lecturas fallidas SEGUIDAS para cortar la ventana  */
/* Pines del bus I2C1, para destrabarlo a mano si un sensor deja SDA en bajo */
#define BOLO_I2C_PORT       GPIOB
#define BOLO_I2C_SCL_PIN    GPIO_PIN_6     /* D5 */
#define BOLO_I2C_SDA_PIN    GPIO_PIN_7     /* D4 */
/* =========================================================
 *  9. WATCHDOG (IWDG) Y PRUEBAS DE FALLA
 * ========================================================= */
/* CubeMX: prescaler 256, reload 3124 -> (3124+1)*256/32000 = 25 s nominal.
 * El LSI no es preciso: en el peor caso el timeout baja a ~23 s. */
#define IWDG_TIMEOUT_MIN_S  23U
/* El watchdog se refresca una vez por ciclo: el ciclo entero (con margen para
 * reintentos e impresion) tiene que entrar en el timeout. Si subis T_REPOSO_S
 * (paso 4, reposos largos) esto deja de compilar a proposito. */
_Static_assert((T_REPOSO_S + T_SENSADO_S + 3U) <= IWDG_TIMEOUT_MIN_S,
        "El ciclo no entra en el watchdog: refrescarlo durante el reposo");

/* Marca que deja el HardFault_Handler en un registro de backup del RTC antes de
 * resetear (sobrevive al reset): asi el arranque sabe que fue un HardFault. */
#define BOLO_MARCA_HARDFAULT  0xBAD0FA17U

/* Prueba de fallas a proposito (dejar en 0 en uso normal):
 * 0 = nada | 1 = cuelgue en el ciclo 3 (lo saca el watchdog) | 2 = HardFault en el ciclo 3 */
#define BOLO_PRUEBA_FALLA   0
/* =========================================================
 *  10. EVENTOS TERMICOS (thermal.c)
 * ========================================================= */
/* Tiempos en SEGUNDOS: el codigo los pasa a ciclos segun T_CICLO_S. Para el bolo
 * final se ajustan estos valores (o T_CICLO_S) y nada mas.
 *   Referencia (bolos reales): una toma de agua baja ~2,3 C en promedio y la
 *   recuperacion mas lenta medida fue ~103 min (22,7 L de agua a 1,7 C). */
#define TERM_TAU_BASE_S           240U  /* la base se acomoda en ~4 min (bolo final: ~2-3 h)   */
#define TERM_EVENTO_MAX_S         600U  /* tope de un evento: 10 min    (bolo final: ~2-3 h)   */
#define TERM_APRENDIZAJE_CICLOS   4U    /* mediciones promediadas para la base inicial          */
#define TERM_UMBRAL_BAJADA_CENTI  100U  /* 1,00 C bajo la base -> BAJADA (posible toma de agua) */
#define TERM_UMBRAL_SUBA_CENTI    100U  /* 1,00 C sobre la base -> SUBA. SOLO DEMO (soplido):
                                         * la fiebre real sube en horas y esta base la seguiria;
                                         * detectarla necesita una base de dias o un umbral
                                         * absoluto. Queda como trabajo futuro. */
#define TERM_HISTERESIS_CENTI     30U   /* BAJADA termina con delta > -0,30 C; SUBA con < +0,30 C */

/* Derivados (no tocar) */
#define TERM_N_BASE             ((TERM_TAU_BASE_S + T_CICLO_S / 2U) / T_CICLO_S)   /* PRUEBA: 16 */
#define TERM_EVENTO_MAX_CICLOS  ((TERM_EVENTO_MAX_S + T_CICLO_S - 1U) / T_CICLO_S) /* PRUEBA: 40 */
_Static_assert(TERM_N_BASE >= 2U,            "TERM_TAU_BASE_S muy corto para el periodo del ciclo");
_Static_assert(TERM_EVENTO_MAX_CICLOS >= 2U, "TERM_EVENTO_MAX_S muy corto para el periodo del ciclo");
_Static_assert(TERM_HISTERESIS_CENTI < TERM_UMBRAL_BAJADA_CENTI &&
               TERM_HISTERESIS_CENTI < TERM_UMBRAL_SUBA_CENTI,
        "La histeresis tiene que ser menor que los umbrales");
/* =========================================================
 *  11. RADIO LoRa (los parametros de radio viven en radio.c, iguales en TX y RX)
 * ========================================================= */
#define BOLO_NODO_ID        1U      /* identifica a este bolo en el paquete (1..255) */

/* =========================================================
 *  12. AHORRO DE ENERGIA (bolo_power.c)
 * ========================================================= */
/* 1 = el debugger sigue conectado mientras el micro duerme en STOP2 (comodo para
 *     depurar paso a paso, pero mantiene relojes prendidos: cientos de uA mas).
 * 0 = consumo real. Para MEDIR consumo, siempre en 0. */
#define BOLO_DBG_EN_STOP    0

#endif /* BOLO_CONFIG_H */
