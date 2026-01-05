#include "system/diagnostics/diagnosticsUART.h" // Header for UART diagnostics

/**
 * @brief Runs diagnostics on all available UART ports.
 *        Currently includes Serial1 and Serial2.
 */
void diagnoseAllUART() {
    standardMessage("Starting UART communication diagnostic", __FILE__, __FUNCTION__, __DATE__, __TIME__);

    diagnoseSerial1();
    diagnoseSerial2();

    standardHeaderFull("Completed UART diagnostic", __FILE__, __FUNCTION__, __DATE__, __TIME__);
}

/**
 * @brief Performs a diagnostic on a specific UART stream.
 *        Waits for incoming data for 3 seconds and prints it if available.
 * 
 * @param serialPort Reference to the HardwareSerial object (e.g., Serial1, Serial2).
 * @param portName Name of the UART port for display purposes.
 */
void diagnoseUARTStream(HardwareSerial& serialPort, const char* portName) {
    Serial.println("📡 Diagnosing " + String(portName));

    serialPort.begin(9600);
    delay(100);

    Serial.println("⌛ Waiting for data on " + String(portName) + " for 3 seconds...");

    unsigned long start = millis();
    bool received = false;

    while (millis() - start < 3000) {
        if (serialPort.available()) {
            received = true;
            Serial.print("📥 Data received on ");
            Serial.print(portName);
            Serial.print(": ");
            while (serialPort.available()) {
                char c = serialPort.read();
                Serial.print(c);
            }
            Serial.println();
            break;
        }
    }

    if (!received) {
        Serial.println("🔻 No data detected on " + String(portName));
    }

    // Do not call serialPort.end() to keep the port active
    Serial.println("✅ End of diagnostic.\n");
}

/**
 * @brief Runs diagnostic on Serial1 (UART1), if available.
 */
void diagnoseSerial1() {
#if defined(HAVE_HWSERIAL1)
    diagnoseUARTStream(Serial1, "Serial1");
#endif
}

/**
 * @brief Runs diagnostic on Serial2 (UART2), if available.
 */
void diagnoseSerial2() {
#if defined(HAVE_HWSERIAL2)
    diagnoseUARTStream(Serial2, "Serial2");
#endif
}
