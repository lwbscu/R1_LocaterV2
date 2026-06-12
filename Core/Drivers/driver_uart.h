#ifndef DRIVER_UART_H
#define DRIVER_UART_H

#include "main.h"
#include <stdint.h>

typedef void (*Driver_UART_LineCallback_t)(const uint8_t *data, uint16_t len);

void Driver_UART_Init(void);
void Driver_UART_RegisterDebugCallback(Driver_UART_LineCallback_t callback);

void Driver_UART_DebugTransmit(const uint8_t *data, uint16_t len);
void Driver_UART_IMUTransmit(const uint8_t *data, uint16_t len);
void Driver_UART_ExtTransmit(const uint8_t *data, uint16_t len);

#endif /* DRIVER_UART_H */
