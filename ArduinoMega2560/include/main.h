#ifndef MAIN_H
#define MAIN_H

#include "system/pinout/pinout.h"
#include <avr8-stub.h>                                              // AVR8 debugging stub for GDB
#include <HardwareSerial.h>                                         // Serial communication support
#include "system/diagnostics/diagnosticsUART.h"                     // UART diagnostics functions
#include "system/diagnostics/diagnosticsEEPROM.h"
#include "system/config/config.h"                                   // System configuration parameters
#include "system/pinout/pinout.h"                                   // Pinout definitions

// Firmware metadata =============================================================================================================================
#define FIRMWARE_VERSION                 "1.0.B"                                    // Firmware version
#define FIRMWARE_NAME                    "ArduinoMega2560"                          // Firmware name
#define FIRMWARE_DATE                    "17/12/2025"                               // Firmware compilation date
#define FIRMWARE_AUTHOR                  "Eduardo Jimenez Serrato"                  // Firmware author
#define FIRMWARE_VERSION_APP             "N.A"                                      // Firmware version for application
#define FIRMWARE_NAME_APP                "N.A"                                      // Firmware name
#define FIRMWARE_DATE_APP                "N.A"                                      // Firmware compilation date

#endif  // MAIN_H