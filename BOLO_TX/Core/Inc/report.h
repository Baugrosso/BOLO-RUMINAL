/**
 * @file    report.h
 * @brief   Salida de informacion del nodo (hoy UART/PuTTY, a futuro tambien LoRa).
 */
#ifndef REPORT_H
#define REPORT_H

#include "main.h"
#include "bolo_types.h"
#include "radio.h"
#include <stdbool.h>

void        report_init(UART_HandleTypeDef *huart);
void        report_banner(void);
void        report_i2c(const uint8_t *dirs, uint8_t n);
void        report_reinicio(const char *causa, bool anomalo);
void        report_prueba_falla(uint8_t tipo);
void        report_init_sensor(const char *nombre, bool ok, uint8_t id);
void        report_sensando(uint32_t ciclo);
void        report_resultado(const bolo_data_t *d);
void        report_durmiendo(uint32_t segundos);
const char *report_estado_str(bolo_estado_t e);
void        report_radio(const radio_diag_t *r);
void        report_radio_tx(const char *msg, const radio_tx_t *t);

#endif /* REPORT_H */
