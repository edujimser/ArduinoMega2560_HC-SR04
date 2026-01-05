#ifndef     CONF_MAIN_H
#define     CONF_MAIN_H

#include <Arduino.h>

/**
 * @brief Main configuration structure for the project.
 *
 * Groups global parameters that control the overall behavior
 * of the application, allowing features to be enabled or disabled
 * at compile-time or runtime.
 */
struct configuracionMain { 
     // BYTE 0 (bits 0–7)
    uint8_t debugMode            : 1; // Bit 0 (LSB)
    uint8_t fullDiagnosticsPins  : 1; // Bit 1
    uint8_t diagnoseAnalog       : 1; // Bit 2
    uint8_t diagnoseGPIO         : 1; // Bit 3
    uint8_t diagnosePWM          : 1; // Bit 4
    uint8_t diagnoseUART         : 1; // Bit 5
    uint8_t diagnoseEEPROM       : 1; // Bit 6
    uint8_t version              : 1; // Bit 7 (MSB)


};
extern configuracionMain systemConfiguration;

#endif // CONF_MAIN_H