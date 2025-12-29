// Include the necessary headers
#include <Arduino.h>
#include "system/pinout/pinout.h"



/**
 * @brief Runs a full diagnostic of digital GPIO and PWM pins.
 * 
 * Calls individual diagnostic functions for GPIO and PWM,
 * printing results via Serial. Useful as an initial hardware test.
 * 
 * @note Ideal for checking general pin status at program startup.
 */
void fullDiagnosticsPins() {
    standardMessage("Starting PINOUT diagnostic", __FILE__, __FUNCTION__, __DATE__, __TIME__);

    diagnoseAnalog(); // ANALOG pin diagnostic
    diagnoseGPIO();   // GPIO pin diagnostic
    diagnosePWM();    // PWM pin diagnosticº

    standardHeaderFull("Full diagnostic complete.", __FILE__, __FUNCTION__, __DATE__, __TIME__);
};


/**
 * @brief Diagnostica el voltaje presente en los pines analógicos del sistema.
 *
 * Esta función recorre todos los pines definidos como analógicos en la estructura `Pins::ANALOG`,
 * configura cada uno como entrada, permite que la señal se estabilice, y luego realiza una lectura
 * analógica. Convierte el valor leído en un voltaje (asumiendo una referencia de 5V) y lo muestra
 * por el puerto serie junto con el nombre y número del pin.
 *
 * @note Utiliza una pequeña demora (5 ms) para permitir la estabilización de la señal antes de la lectura.
 * @warning Asegúrate de que los pines estén correctamente definidos en `Pins::ANALOG` y que el sistema
 *          esté usando una referencia de voltaje de 5V para que la conversión sea precisa.
 *
 * @see Pins::ANALOG
 * @see analogRead()
 * @see pinMode()
 */

void diagnoseAnalog() {
    standardHeaderFull("⚡ Detecting external voltage on ANALOG pins:", __FILE__, __FUNCTION__, __DATE__, __TIME__);

    for (size_t i = 0; i < Pins::NUM_ANALOG; ++i) {
        const PinInfo& pin = Pins::ANALOG[i];

        //Forze the pin to OUTPUT for redux noise and stabily electric system
        pinMode(pin.number, OUTPUT); 
        digitalWrite(pin.number, LOW); 
        delay(20);                   

        //Read the pin as INPUT to detect voltage
        pinMode(pin.number, INPUT); 
        delay(20);                   

        int value = analogRead(pin.number); // Read analog value
        delay(20);
        float voltage = value * (5.0 / 1023.0); // Convert to voltage (assuming 5V reference)

        //Forze the pin to OUTPUT for redux noise and stabily electric system
        pinMode(pin.number, OUTPUT); 
        digitalWrite(pin.number, LOW);
        delay(20);

        //State for default pin for init system
        pinMode(pin.number, INPUT);
        delay(20);

        Serial.print("• ");
        Serial.print(pin.name);
        Serial.print(" [#");
        Serial.print(pin.number);
        Serial.print("] → Voltage: ");
        Serial.print(voltage,2);
        Serial.println(" V");
        
    };
};

/**
 * @brief Diagnostic for digital GPIO pins configured as input with pull-up resistor.
 * 
 * Iterates through all defined GPIO pins, configures them as INPUT_PULLUP,
 * and checks if they are connected to ground (LOW reading). Prints results via Serial.-
 * 
 * @note Useful for detecting if a pin is grounded.
 */
void diagnoseGPIO(){
    standardHeaderFull("⚡ Detecting external voltage on GPIO pins:", __FILE__, __FUNCTION__, __DATE__, __TIME__);

    for (size_t i = 0; i < Pins::NUM_GPIO; ++i) {
        const PinInfo& pin = Pins::GPIO[i];

        //Forze the pin to OUTPUT for redux noise and stabily electric system
        pinMode(pin.number, OUTPUT); 
        digitalWrite(pin.number, LOW); 
        delay(20); 

        pinMode(pin.number, INPUT);         // Set pin as input without pull-up
        delay(5);                           // Allow signal to stabilize

        int state = digitalRead(pin.number); // Read pin state

        Serial.print("• ");
        Serial.print(pin.name);
        Serial.print(" [#");
        Serial.print(pin.number);
        Serial.print("] → Voltage: ");

        if (state == HIGH) {
            Serial.println("⚡ External voltage detected (HIGH)");
        } else {
            Serial.println("🔻 No voltage (LOW or connected to GND)");
        }
    }

}


/**
 * @brief Detects the presence of external voltage on PWM-capable pins.
 *
 * This function loops through all defined PWM pins and configures each one as an input
 * (without enabling the internal pull-up resistor). It then reads the digital state of the pin
 * to determine whether external voltage is present.
 *
 * A brief delay is added after setting the pin mode to allow the signal to stabilize.
 * The result of each pin check is printed to the Serial Monitor, indicating whether
 * voltage was detected (HIGH) or not (LOW or connected to ground).
 *
 * Note: This function does not measure the actual voltage level. It only detects
 * logical HIGH or LOW states. For precise voltage measurement, analog pins and analogRead()
 * should be used instead.
 *
 * Useful for:
 * - Verifying electrical signals on PWM pins
 * - Debugging hardware connections
 * - Checking for floating or grounded pins
 */
void diagnosePWM() {
// Display a formatted header with file, function, date, and time
    standardHeaderFull("⚡ Detecting voltage on PWM pins:", __FILE__, __FUNCTION__, __DATE__, __TIME__);

    // Loop through all defined PWM pins
    for (size_t i = 0; i < Pins::NUM_PWM; ++i) {
        const PinInfo& pin = Pins::PWM[i];  // Access current pin info

        //Forze the pin to OUTPUT for redux noise and stabily electric system
        pinMode(pin.number, OUTPUT); 
        analogWrite(pin.number, 0); 
        delay(20); 

        pinMode(pin.number, INPUT);         // Set pin as input (no pull-up)
        delay(5);                           // Brief delay to stabilize reading

        int state = digitalRead(pin.number); // Read the electrical state of the pin

        // Print diagnostic result to Serial Monitor
        Serial.print("• ");
        Serial.print(pin.name);
        Serial.print(" [Pin ");
        Serial.print(pin.number);
        Serial.print("] → Voltage: ");

        if (state == HIGH) {
            Serial.println("⚡ Voltage detected (HIGH)");
        } else {
            Serial.println("🔻 No voltage (LOW or connected to GND)");
        };
    };
};


/*
 * Functions to map pin names or numbers to their physical identifiers across multiple interfaces
 * Implemented in pins.cpp
 */

uint8_t pinNumber(const PinInfo pin) {
    return pin.number;
}

const char* pinName(const PinInfo pin) {
    return pin.name;
}

const char* pinFamily(const PinInfo pin) {
    return pin.family;
}

/* 
* Returns true if the pin is a valid pin 
*/
bool isValidGPIO(const PinInfo& pin) {
    for (size_t i = 0; i < Pins::NUM_GPIO; ++i) {
        if (Pins::GPIO[i].number == pin.number &&
            Pins::GPIO[i].family == pin.family) 
        {
            return true;
        }
    }

    return false;
};

/* 
* Returns true if the pin is a valid pin 
*/
bool isValidPWM(const PinInfo& pin) {
    for (size_t i = 0; i < Pins::NUM_PWM; ++i) {
        if (Pins::PWM[i].number == pin.number &&
            Pins::PWM[i].family == pin.family) 
        {
            return true;
        }
    }

    return false;
};

/* 
* Returns true if the pin is a valid pin 
*/
bool isValidAnalog(const PinInfo& pin) {
    for (size_t i = 0; i < Pins::NUM_ANALOG; ++i) {
        if (Pins::ANALOG[i].number == pin.number &&
            Pins::ANALOG[i].family == pin.family) 
        {
            return true;
        }
    }

    return false;
};