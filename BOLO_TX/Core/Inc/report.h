/**
 * @file    report.h
 * @brief   Salida de informacion por UART (PuTTY, 115200 8N1). Solo depuracion:
 *          con BOLO_LOG = 0 no se imprime nada (consumo real).
 */
#ifndef REPORT_H
#define REPORT_H

#include "main.h"
#include "bolo_types.h"
#include "radio.h"
#include "contacto.h"
#include "periodo.h"
#include "almacen.h"
#include <stdbool.h>

void report_init(UART_HandleTypeDef *huart);
void report_banner(void);
void report_reinicio(const char *causa, bool anomalo);
void report_i2c(const uint8_t *dirs, uint8_t n);
void report_crc(bool hw_ok);
void report_sensor(const char *nombre, bool ok, uint8_t id);
void report_radio(const radio_diag_t *r);
void report_almacen(const alm_estado_t *e, bool recuperado);
void report_hora(uint32_t t_unix, int32_t delta, bool primera);
void report_epoca(uint32_t t, const epoca_t *e);
void report_th(uint32_t t, bool ok, int16_t t_centi, uint16_t h_centi);
void report_periodo(const periodo_info_t *p, uint16_t vdd_mv, bool parcial);
void report_contacto_inicio(uint8_t contacto, uint8_t intento, bool pide_ranura);
void report_contacto_fin(const cont_res_t *r);
void report_ranura(uint8_t ranura);
void report_potencia(int8_t dbm);
void report_proximo(uint32_t ahora, uint32_t t_contacto);

#endif /* REPORT_H */
