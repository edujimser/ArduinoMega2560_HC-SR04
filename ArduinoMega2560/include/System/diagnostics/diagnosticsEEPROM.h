#ifndef DIAGNOSTICSEEPROM_H
#define DIAGNOSTICSEEPROM_H

#include <Arduino.h>
#include <EEPROM.h>
#include "system/msg/msg.h" // Message handling functions

/**
 * @brief Provides diagnostic utilities for testing and managing EEPROM memory.
 */
class DiagnosticsEEPROM {
public:
    /**
     * @brief Runs a diagnostic test on the EEPROM at the specified address.
     *        Writes a test value, reads it back, and verifies integrity.
     * 
     * @param address EEPROM address to test (default is 0).
     * @return true if EEPROM responds correctly, false otherwise.
     */
    static bool runTest(int address = 0);

    /**
     * @brief Estimates the amount of free RAM available on the system.
     *        Works on AVR-based boards like Arduino Mega 2560.
     * 
     * @return Number of free bytes in RAM.
     */
    static int getFreeMemory();

private:
    /**
     * @brief Value used for EEPROM testing.
     */
    static const byte testValue = 123;

    /**
     * @brief Clears the EEPROM at the specified address by writing 0xFF.
     * 
     * @param address EEPROM address to clear.
     */
    static void clearEEPROM(int address);
};

/**
 * @brief [Deprecated] Global version of getFreeMemory().
 *        Prefer using DiagnosticsEEPROM::getFreeMemory() instead.
 * 
 * @return Number of free bytes in RAM.
 */
int getFreeMemory();

#endif // DIAGNOSTICSEEPROM_H
