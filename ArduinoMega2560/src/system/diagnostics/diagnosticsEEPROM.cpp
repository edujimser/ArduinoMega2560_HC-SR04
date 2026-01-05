#include "system/diagnostics/diagnosticsEEPROM.h"

/**
 * @brief Runs a diagnostic test on the EEPROM at the specified address.
 *        Writes a test value, reads it back, compares, and reports memory status.
 * 
 * @param address EEPROM address to test (default is 0).
 * @return true if EEPROM responds correctly, false otherwise.
 */
bool DiagnosticsEEPROM::runTest(int address) {
    standardMessage("🧪 Starting EEPROM diagnostic", __FILE__, __FUNCTION__, __DATE__, __TIME__);

    // Write test value
    EEPROM.write(address, testValue);
    delay(10);

    // Read stored value
    byte readValue = EEPROM.read(address);
    Serial.print("📥 Value read from EEPROM[");
    Serial.print(address);
    Serial.print("]: ");
    Serial.println(readValue);

    // Show memory status
    int freeMem = getFreeMemory();
    Serial.print("📊 Estimated free memory: ");
    Serial.print(freeMem);
    Serial.println(" bytes");

    Serial.print("📦 Estimated used memory: ");
    Serial.print(RAMEND - freeMem);
    Serial.println(" bytes");

    // Check if value matches
    if (readValue == testValue) {
        Serial.println("✅ EEPROM is responding correctly.");
        clearEEPROM(address);
        standardHeaderFull("EEPROM diagnostic completed", __FILE__, __FUNCTION__, __DATE__, __TIME__);
        return true;
    } else {
        Serial.println("❌ EEPROM error: value mismatch.");
        clearEEPROM(address);
        standardHeaderFull("EEPROM diagnostic failed", __FILE__, __FUNCTION__, __DATE__, __TIME__);
        return false;
    }
}


/**
 * @brief Clears the EEPROM at the specified address by writing 0xFF.
 * 
 * @param address EEPROM address to clear.
 */
void DiagnosticsEEPROM::clearEEPROM(int address) {
    EEPROM.write(address, 0xFF);
    delay(10);

    byte clearedValue = EEPROM.read(address);
    Serial.print("🧹 EEPROM cleared. New value at [");
    Serial.print(address);
    Serial.print("]: ");
    Serial.println(clearedValue);
}


/**
 * @brief Estimates the amount of free RAM available on the system.
 *        Works on AVR-based boards like Arduino Mega 2560.
 * 
 * @return Number of free bytes in RAM.
 */
int DiagnosticsEEPROM::getFreeMemory() {
    extern unsigned int __heap_start;
    extern void *__brkval;
    int freeMemory;

    if ((int)__brkval == 0) {
        freeMemory = ((int)&freeMemory) - ((int)&__heap_start);
    } else {
        freeMemory = ((int)&freeMemory) - ((int)__brkval);
    }

    return freeMemory;
}



