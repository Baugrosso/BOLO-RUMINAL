/* stm32l4xx_hal.h de mentira para compilar la logica del firmware en la PC (BOLO_HOST).
 * Solo trae los tipos de la HAL que nombran los .h de los drivers. */
#ifndef HAL_STUB_H
#define HAL_STUB_H
#include <stdint.h>
typedef struct { int dummy; } I2C_HandleTypeDef;
typedef struct { int dummy; } SPI_HandleTypeDef;
typedef struct { int dummy; } UART_HandleTypeDef;
typedef struct { int dummy; } RTC_HandleTypeDef;
typedef struct { int dummy; } IWDG_HandleTypeDef;
#endif
