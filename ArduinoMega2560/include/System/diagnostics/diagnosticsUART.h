#ifndef DIAGNOSTICS_UART_H
#define DIAGNOSTICS_UART_H

#include <Arduino.h>            // Arduino core library for basic functions
#include "system/msg/msg.h"            // Message handling functions

/**
 * @brief Runs a complete diagnostic on all available UART ports.
 */
void diagnoseAllUART();

/**
 * @brief Performs a diagnostic on Serial1 (UART1).
 */
void diagnoseSerial1();

/**
 * @brief Performs a diagnostic on Serial2 (UART2).
 */
void diagnoseSerial2();

#endif // DIAGNOSTICS_UART_H
