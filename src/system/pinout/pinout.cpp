// Include the necessary headers
#include <Arduino.h>
#include "system/pinout/pinout.h"

void initPins() {
    // UART: Asynchronous serial communication
    // Variable                                | Pin array                              | Descripción
    // ----------------------------------------|----------------------------------------|----------------------------------------
    [[maybe_unused]] PinInfo uartRx0  =         Pins::UART_RX[0];                        // RX0   → UART0 via USB                
    [[maybe_unused]] PinInfo uartRx1  =         Pins::UART_RX[1];                        // RX1   → UART1 + INT4
    [[maybe_unused]] PinInfo uartRx2  =         Pins::UART_RX[2];                        // RX2   → UART2
    [[maybe_unused]] PinInfo uartRx3  =         Pins::UART_RX[3];                        // RX3   → UART3
    [[maybe_unused]] PinInfo uartTx0  =         Pins::UART_TX[0];                        // TX0   → UART0 via USB
    [[maybe_unused]] PinInfo uartTx1  =         Pins::UART_TX[1];                        // TX1   → UART1 + INT5
    [[maybe_unused]] PinInfo uartTx2  =         Pins::UART_TX[2];                        // TX2   → UART2
    [[maybe_unused]] PinInfo uartTx3  =         Pins::UART_TX[3];                        // TX3   → UART3

    // SPI: Synchronous master-slave communication
    // Variable                                | Pin array                              | Descripción
    // ----------------------------------------|----------------------------------------|----------------------------------------
    [[maybe_unused]] PinInfo spiMiso  =         Pins::SPI[0];                            // MISO  → pin 50
    [[maybe_unused]] PinInfo spiMosi  =         Pins::SPI[1];                            // MOSI  → pin 51
    [[maybe_unused]] PinInfo spiSck   =         Pins::SPI[2];                            // SCK   → pin 52
    [[maybe_unused]] PinInfo spiSs    =         Pins::SPI[3];                            // SS    → pin 53

    // I2C / TWI: Two-wire communication
    [[maybe_unused]] PinInfo i2cSda   =         Pins::I2C[0];                            // SDA   → pin 20
    [[maybe_unused]] PinInfo i2cScl   =         Pins::I2C[1];                            // SCL   → pin 21

    // External interrupts
    // Variable                                | Pin array                              | Descripción
    // ----------------------------------------|----------------------------------------|----------------------------------------
    [[maybe_unused]] PinInfo int0     =         Pins::INTERRUPTS[0];                     // INT0  → pin 2
    [[maybe_unused]] PinInfo int1     =         Pins::INTERRUPTS[1];                     // INT1  → pin 3
    [[maybe_unused]] PinInfo int2     =         Pins::INTERRUPTS[2];                     // INT2  → pin 21
    [[maybe_unused]] PinInfo int3     =         Pins::INTERRUPTS[3];                     // INT3  → pin 20
    [[maybe_unused]] PinInfo int4     =         Pins::INTERRUPTS[4];                     // INT4  → pin 19
    [[maybe_unused]] PinInfo int5     =         Pins::INTERRUPTS[5];                     // INT5  → pin 18

    // ANALOG: Pins with analog input capability (ADC)
    // Variable                                | Pin array                              | Descripción             | Set pinMode   
    // ----------------------------------------|----------------------------------------|----------------------------------------------------------------
    [[maybe_unused]] PinInfo analog0  =         Pins::ANALOG[0];                         /* A0    → pin 54*/        pinMode(analog0.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog1  =         Pins::ANALOG[1];                         /* A1    → pin 55*/        pinMode(analog1.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog2  =         Pins::ANALOG[2];                         /* A2    → pin 56*/        pinMode(analog2.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog3  =         Pins::ANALOG[3];                         /* A3    → pin 57*/        pinMode(analog3.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog4  =         Pins::ANALOG[4];                         /* A4    → pin 58*/        pinMode(analog4.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog5  =         Pins::ANALOG[5];                         /* A5    → pin 59*/        pinMode(analog5.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog6  =         Pins::ANALOG[6];                         /* A6    → pin 60*/        pinMode(analog6.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog7  =         Pins::ANALOG[7];                         /* A7    → pin 61*/        pinMode(analog7.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog8  =         Pins::ANALOG[8];                         /* A8    → pin 62*/        pinMode(analog8.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog9  =         Pins::ANALOG[9];                         /* A9    → pin 63*/        pinMode(analog9.number,  OUTPUT);
    [[maybe_unused]] PinInfo analog10 =         Pins::ANALOG[10];                        /* A10   → pin 64*/        pinMode(analog10.number, OUTPUT);
    [[maybe_unused]] PinInfo analog11 =         Pins::ANALOG[11];                        /* A11   → pin 65*/        pinMode(analog11.number, OUTPUT);
    [[maybe_unused]] PinInfo analog12 =         Pins::ANALOG[12];                        /* A12   → pin 66*/        pinMode(analog12.number, OUTPUT);
    [[maybe_unused]] PinInfo analog13 =         Pins::ANALOG[13];                        /* A13   → pin 67*/        pinMode(analog13.number, OUTPUT);
    [[maybe_unused]] PinInfo analog14 =         Pins::ANALOG[14];                        /* A14   → pin 68*/        pinMode(analog14.number, OUTPUT);
    [[maybe_unused]] PinInfo analog15 =         Pins::ANALOG[15];                        /* A15   → pin 69*/        pinMode(analog15.number, OUTPUT);
    // PWM: Pins with pulse-width modulation capability
    // Variable                                | Pin array                              | Descripción             | Set pinMode   
    // ----------------------------------------|----------------------------------------|----------------------------------------------------------------
    [[maybe_unused]] PinInfo pwm0     =         Pins::PWM[0];                            /* PWM02 → pin 2*/         pinMode(pwm0.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm1     =         Pins::PWM[1];                            /* PWM03 → pin 3*/         pinMode(pwm1.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm2     =         Pins::PWM[2];                            /* PWM04 → pin 4*/         pinMode(pwm2.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm3     =         Pins::PWM[3];                            /* PWM05 → pin 5*/         pinMode(pwm3.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm4     =         Pins::PWM[4];                            /* PWM06 → pin 6*/         pinMode(pwm4.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm5     =         Pins::PWM[5];                            /* PWM07 → pin 7*/         pinMode(pwm5.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm6     =         Pins::PWM[6];                            /* PWM08 → pin 8*/         pinMode(pwm6.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm7     =         Pins::PWM[7];                            /* PWM09 → pin 9*/         pinMode(pwm7.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm8     =         Pins::PWM[8];                            /* PWM10 → pin 10*/        pinMode(pwm8.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm9     =         Pins::PWM[9];                            /* PWM11 → pin 11*/        pinMode(pwm9.number,  OUTPUT);
    [[maybe_unused]] PinInfo pwm10    =         Pins::PWM[10];                           /* PWM12 → pin 12*/        pinMode(pwm10.number, OUTPUT);
    [[maybe_unused]] PinInfo pwm11    =         Pins::PWM[11];                           /* PWM44 → pin 44*/        pinMode(pwm11.number, OUTPUT);
    [[maybe_unused]] PinInfo pwm12    =         Pins::PWM[12];                           /* PWM45 → pin 45*/        pinMode(pwm12.number, OUTPUT);
    [[maybe_unused]] PinInfo pwm13    =         Pins::PWM[13];                           /* PWM46 → pin 46*/        pinMode(pwm13.number, OUTPUT);

    // GPIO: General-purpose digital pins
    // Variable                                | Pin array                              | Descripción             | Set pinMode   
    // ----------------------------------------|----------------------------------------|----------------------------------------------------------------
    [[maybe_unused]] PinInfo gpio0    =         Pins::GPIO[0];                           /* GPIO13 → pin 13 */      pinMode(gpio0.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio1    =         Pins::GPIO[1];                           /* GPIO22 → pin 22 */      pinMode(gpio1.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio2    =         Pins::GPIO[2];                           /* GPIO23 → pin 23 */      pinMode(gpio2.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio3    =         Pins::GPIO[3];                           /* GPIO24 → pin 24 */      pinMode(gpio3.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio4    =         Pins::GPIO[4];                           /* GPIO25 → pin 25 */      pinMode(gpio4.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio5    =         Pins::GPIO[5];                           /* GPIO26 → pin 26 */      pinMode(gpio5.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio6    =         Pins::GPIO[6];                           /* GPIO27 → pin 27 */      pinMode(gpio6.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio7    =         Pins::GPIO[7];                           /* GPIO28 → pin 28 */      pinMode(gpio7.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio8    =         Pins::GPIO[8];                           /* GPIO29 → pin 29 */      pinMode(gpio8.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio9    =         Pins::GPIO[9];                           /* GPIO30 → pin 30 */      pinMode(gpio9.number,  OUTPUT);
    [[maybe_unused]] PinInfo gpio10   =         Pins::GPIO[10];                          /* GPIO31 → pin 31 */      pinMode(gpio10.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio11   =         Pins::GPIO[11];                          /* GPIO32 → pin 32 */      pinMode(gpio11.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio12   =         Pins::GPIO[12];                          /* GPIO33 → pin 33 */      pinMode(gpio12.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio13   =         Pins::GPIO[13];                          /* GPIO34 → pin 34 */      pinMode(gpio13.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio14   =         Pins::GPIO[14];                          /* GPIO35 → pin 35 */      pinMode(gpio14.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio15   =         Pins::GPIO[15];                          /* GPIO36 → pin 36 */      pinMode(gpio15.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio16   =         Pins::GPIO[16];                          /* GPIO37 → pin 37 */      pinMode(gpio16.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio17   =         Pins::GPIO[17];                          /* GPIO38 → pin 38 */      pinMode(gpio17.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio18   =         Pins::GPIO[18];                          /* GPIO39 → pin 39 */      pinMode(gpio18.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio19   =         Pins::GPIO[19];                          /* GPIO40 → pin 40 */      pinMode(gpio19.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio20   =         Pins::GPIO[20];                          /* GPIO41 → pin 41 */      pinMode(gpio20.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio21   =         Pins::GPIO[21];                          /* GPIO42 → pin 42 */      pinMode(gpio21.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio22   =         Pins::GPIO[22];                          /* GPIO43 → pin 43 */      pinMode(gpio22.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio23   =         Pins::GPIO[23];                          /* GPIO44 → pin 44 */      pinMode(gpio23.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio24   =         Pins::GPIO[24];                          /* GPIO45 → pin 45 */      pinMode(gpio24.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio25   =         Pins::GPIO[25];                          /* GPIO46 → pin 46 */      pinMode(gpio25.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio26   =         Pins::GPIO[26];                          /* GPIO47 → pin 47 */      pinMode(gpio26.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio27   =         Pins::GPIO[27];                          /* GPIO48 → pin 48 */      pinMode(gpio27.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio28   =         Pins::GPIO[28];                          /* GPIO49 → pin 49 */      pinMode(gpio28.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio29   =         Pins::GPIO[29];                          /* GPIO50 → pin 50 */      pinMode(gpio29.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio30   =         Pins::GPIO[30];                          /* GPIO51 → pin 51 */      pinMode(gpio30.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio31   =         Pins::GPIO[31];                          /* GPIO52 → pin 52 */      pinMode(gpio31.number, OUTPUT);
    [[maybe_unused]] PinInfo gpio32   =         Pins::GPIO[32];                          /* GPIO53 → pin 53 */      pinMode(gpio32.number, OUTPUT);

}

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