#include <Arduino.h>
#include "main.h"

void setup() {                                                 // Arduino setup function (runs once at startup)

                                                               // Otherwise, run in normal execution mode
    Serial.begin(57600);                                       // start serial communication at 57600 baud
    while (!Serial);                                       

    //Config System
    configuracionMain systemConfiguration = {
                                                                                                             // Byte 0 (bits 0–7)
        .debugMode = false,                                    // Enable debug mode                          | Byte 0, Bit 0 (LSB)
        .fullDiagnosticsPins = true,                           // Run full pin diagnostics                   | Byte 0, Bit 1
        .diagnoseAnalog = false,                               // Disable analog diagnostics                 | Byte 0, Bit 2
        .diagnoseGPIO = false,                                 // Disable GPIO diagnostics                   | Byte 0, Bit 3
        .diagnosePWM = false,                                  // Disable PWM diagnostics                    | Byte 0, Bit 4
        .diagnoseUART = true,                                  // Enable UART diagnostics                    | Byte 0, Bit 5
        .diagnoseEEPROM = true,                                // Disable EEPROM diagnostics                 | Byte 0, Bit 6
        .version = true                                        // Byte 0, Bit 7 → reservado o libre (MSB)    | Byte 0, Bit 7
    };

    if (systemConfiguration.fullDiagnosticsPins) fullDiagnosticsPins();
    if (systemConfiguration.diagnoseAnalog) diagnoseAnalog();
    if (systemConfiguration.diagnoseGPIO) diagnoseGPIO();
    if (systemConfiguration.diagnosePWM)  diagnosePWM(); 
    if (systemConfiguration.diagnoseUART) diagnoseAllUART();
    if (systemConfiguration.diagnoseEEPROM) DiagnosticsEEPROM::runTest();
    if (systemConfiguration.debugMode) debug_init(); 
    if (systemConfiguration.version) printVersion(FIRMWARE_VERSION, FIRMWARE_NAME, FIRMWARE_DATE, FIRMWARE_AUTHOR, FIRMWARE_VERSION_APP, FIRMWARE_NAME_APP, FIRMWARE_DATE_APP);

    initPins();                                                // Initialize all system pins
};



void loop() {
    
} 
