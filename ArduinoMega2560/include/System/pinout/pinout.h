#ifndef PINS_H
#define PINS_H

#include <Arduino.h>
#include "system/msg/msg.h"

/*
 * 📌 Structure to represent pins with symbolic name and physical number
 */
struct PinInfo {
    const char* name;       // Symbolic name of the pin
    int number;             // Physical pin number on the board
    const char* family;     // Pin family or category (e.g., "PWM", "GPIO")
    // Constructor constexpr para permitir uso en evaluaciones de tiempo de compilación
    constexpr PinInfo(const char* n, int num, const char* fam) : name(n), number(num), family(fam) {}

};

/*
 * 📦 Namespace Pins: Groups all pin arrays and basic functions
 */
namespace Pins {

    /*
     * 🔗 UART: Asynchronous serial communication
     */
  inline constexpr PinInfo UART_RX[] = {
        {"PIN_RX0", 0,  "UART_RX"},    // RX0: UART0 via USB (Serial.begin)
        {"PIN_RX1", 19, "UART_RX"},    // RX1: UART1 + INT4 interrupt (external communication)
        {"PIN_RX2", 17, "UART_RX"},    // RX2: UART2 (additional devices)
        {"PIN_RX3", 15, "UART_RX"},    // RX3: UART3 (extended use on Mega)
    };


    inline constexpr PinInfo UART_TX[] = {
        {"PIN_TX0", 1,  "UART_TX"},    // TX0: UART0 via USB (Serial.print)
        {"PIN_TX1", 18, "UART_TX"},    // TX1: UART1 + INT5 interrupt
        {"PIN_TX2", 16, "UART_TX"},    // TX2: UART2
        {"PIN_TX3", 14, "UART_TX"},    // TX3: UART3
    };

    /*
     * 🔄 SPI: Synchronous master-slave communication
     */
    inline constexpr PinInfo SPI[] = {
        {"PIN_MISO", 50, "SPI"},    // MISO: receives data from slave
        {"PIN_MOSI", 51, "SPI"},    // MOSI: sends data to slave
        {"PIN_SCK", 52,  "SPI"},    // SCK: clock signal
        {"PIN_SS",  53,  "SPI"},    // SS/SDA: slave select (must be OUTPUT)
    };

    /*
     * 🔧 I2C / TWI: Two-wire communication
     */
    inline constexpr PinInfo I2C[] = {
        {"PIN_SDA", 20, "I2C"},    // SDA: data line + INT3 interrupt
        {"PIN_SCL", 21, "I2C"},    // SCL: clock line + INT2 interrupt
    };

    /*
     * ⚡ External interrupts
     */
    inline constexpr PinInfo INTERRUPTS[] = {
        {"INT_0", 2,  "INTERRUPTS"},    // INT0: button, sensor, etc.
        {"INT_1", 3,  "INTERRUPTS"},    // INT1: button, sensor, etc.
        {"INT_2", 21, "INTERRUPTS"},    // INT2: shared with SCL
        {"INT_3", 20, "INTERRUPTS"},    // INT3: shared with SDA
        {"INT_4", 19, "INTERRUPTS"},    // INT4: shared with RX1
        {"INT_5", 18, "INTERRUPTS"},    // INT5: shared with TX1
    };

    /*
    * 🎯 ANALOG: Pins with analog input capability (ADC)
    */
    inline constexpr PinInfo ANALOG[] = {
        {"PIN_ANALOG_A00", 54, "ANALOG"},   // A0: general analog input
        {"PIN_ANALOG_A01", 55, "ANALOG"},   // A1: general analog input
        {"PIN_ANALOG_A02", 56, "ANALOG"},   // A2: general analog input
        {"PIN_ANALOG_A03", 57, "ANALOG"},   // A3: general analog input
        {"PIN_ANALOG_A04", 58, "ANALOG"},   // A4: general analog input
        {"PIN_ANALOG_A05", 59, "ANALOG"},   // A5: general analog input
        {"PIN_ANALOG_A06", 60, "ANALOG"},   // A6: general analog input
        {"PIN_ANALOG_A07", 61, "ANALOG"},   // A7: general analog input
        {"PIN_ANALOG_A08", 62, "ANALOG"},   // A8: extended analog input (Mega only)
        {"PIN_ANALOG_A90", 63, "ANALOG"},   // A9: extended analog input (Mega only)
        {"PIN_ANALOG_A10", 64, "ANALOG"},   // A10: extended analog input (Mega only)
        {"PIN_ANALOG_A11", 65, "ANALOG"},   // A11: extended analog input (Mega only)
        {"PIN_ANALOG_A12", 66, "ANALOG"},   // A12: extended analog input (Mega only)
        {"PIN_ANALOG_A13", 67, "ANALOG"},   // A13: extended analog input (Mega only)
        {"PIN_ANALOG_A14", 68, "ANALOG"},   // A14: extended analog input (Mega only)
        {"PIN_ANALOG_A15", 69, "ANALOG"},   // A15: extended analog input (Mega only)
    };

    /*
     * 🌀 PWM: Pins with pulse-width modulation capability
     */
    inline constexpr PinInfo PWM[] = {
        {"PIN_PWM_02", 2,  "PWM"},    // PWM + INT0: useful for buttons with interrupt
        {"PIN_PWM_03", 3,  "PWM"},    // PWM + INT1: servos or sensors with interrupt
        {"PIN_PWM_04", 4,  "PWM"},    // Standard PWM: LED, motor
        {"PIN_PWM_05", 5,  "PWM"},    // Standard PWM: LED, motor
        {"PIN_PWM_06", 6,  "PWM"},    // Standard PWM: LED, motor
        {"PIN_PWM_07", 7,  "PWM"},    // Standard PWM: LED, motor
        {"PIN_PWM_08", 8,  "PWM"},    // Standard PWM: LED, motor
        {"PIN_PWM_09", 9,  "PWM"},    // Standard PWM: LED, motor
        {"PIN_PWM_10", 10, "PWM"},    // PWM + SPI SS: ideal as SPI output
        {"PIN_PWM_11", 11, "PWM"},    // Standard PWM: LED, motor
        {"PIN_PWM_12", 12, "PWM"},    // Standard PWM: LED, motor
        {"PIN_PWM_44", 44, "PWM"},    // Extended PWM (Mega): large motors
        {"PIN_PWM_45", 45, "PWM"},    // Extended PWM (Mega): large motors
        {"PIN_PWM_46", 46, "PWM"}     // Extended PWM (Mega): large motors
    };

    /*
     * 🧩 GPIO: General-purpose digital pins
     */
    inline constexpr PinInfo GPIO[] = {
        {"PIN_GPIO_13", 13, "GPIO"},  // Digital: general input/output PWM13 'Note: this pin is digital' input/output 0
        {"PIN_GPIO_22", 22, "GPIO"},  // Digital: general input/output 1
        {"PIN_GPIO_23", 23, "GPIO"},  // Digital: general input/output 2
        {"PIN_GPIO_24", 24, "GPIO"},  // Digital: general input/output 3
        {"PIN_GPIO_25", 25, "GPIO"},  // Digital: general input/output 4
        {"PIN_GPIO_26", 26, "GPIO"},  // Digital: general input/output 5
        {"PIN_GPIO_27", 27, "GPIO"},  // Digital: general input/output 6
        {"PIN_GPIO_28", 28, "GPIO"},  // Digital: general input/output 7
        {"PIN_GPIO_29", 29, "GPIO"},  // Digital: general input/output 8
        {"PIN_GPIO_30", 30, "GPIO"},  // Digital: general input/output 9
        {"PIN_GPIO_31", 31, "GPIO"},  // Digital: general input/output 10
        {"PIN_GPIO_32", 32, "GPIO"},  // Digital: general input/output 11 
        {"PIN_GPIO_33", 33, "GPIO"},  // Digital: general input/output 12
        {"PIN_GPIO_34", 34, "GPIO"},  // Digital: general input/output 13
        {"PIN_GPIO_35", 35, "GPIO"},  // Digital: general input/output 14
        {"PIN_GPIO_36", 36, "GPIO"},  // Digital: general input/output 15
        {"PIN_GPIO_37", 37, "GPIO"},  // Digital: general input/output 16
        {"PIN_GPIO_38", 38, "GPIO"},  // Digital: general input/output 17
        {"PIN_GPIO_39", 39, "GPIO"},  // Digital: general input/output 18
        {"PIN_GPIO_40", 40, "GPIO"},  // Digital: general input/output 19 
        {"PIN_GPIO_41", 41, "GPIO"},  // Digital: general input/output 20
        {"PIN_GPIO_42", 42, "GPIO"},  // Digital: general input/output 21
        {"PIN_GPIO_43", 43, "GPIO"},  // Digital: general input/output 22
        {"PIN_GPIO_44", 44, "GPIO"},  // Digital: general input/output 23
        {"PIN_GPIO_45", 45, "GPIO"},  // Digital: general input/output 24
        {"PIN_GPIO_46", 46, "GPIO"},  // Digital: general input/output 25
        {"PIN_GPIO_47", 47, "GPIO"},  // Digital: general input/output 26
        {"PIN_GPIO_48", 48, "GPIO"},  // Digital: general input/output 27
        {"PIN_GPIO_49", 49, "GPIO"},  // Digital: general input/output 28
        {"PIN_GPIO_50", 50, "GPIO"},  // Digital: general input/output 29
        {"PIN_GPIO_51", 51, "GPIO"},  // Digital: general input/output 30
        {"PIN_GPIO_52", 52, "GPIO"},  // Digital: general input/output 31
        {"PIN_GPIO_53", 53, "GPIO"},  // Digital: general input/output 32
    };

    /*
     * 📏 Array sizes (for loops and diagnostics)
     */
    constexpr size_t NUM_ANALOG     = sizeof(ANALOG) / sizeof(ANALOG[0]);
    constexpr size_t NUM_GPIO       = sizeof(GPIO) / sizeof(GPIO[0]);
    constexpr size_t NUM_PWM        = sizeof(PWM) / sizeof(PWM[0]);
    constexpr size_t NUM_UART_RX    = sizeof(UART_RX) / sizeof(UART_RX[0]);
    constexpr size_t NUM_UART_TX    = sizeof(UART_TX) / sizeof(UART_TX[0]);
    constexpr size_t NUM_SPI        = sizeof(SPI) / sizeof(SPI[0]);
    constexpr size_t NUM_I2C        = sizeof(I2C) / sizeof(I2C[0]);
    constexpr size_t NUM_INTERRUPTS = sizeof(INTERRUPTS) / sizeof(INTERRUPTS[0]);

}

 
/**
 * @brief Returns the numeric Arduino pin associated with a PinInfo object.
 *
 * @param pin  PinInfo structure containing pin metadata.
 * @return The pin number (0–69 on ATmega2560).
 */
uint8_t pinNumber(const PinInfo pin);

/**
 * @brief Returns the human‑readable name of a pin.
 *
 * @param pin  PinInfo structure containing pin metadata.
 * @return Pointer to a string containing the pin name.
 */
const char* pinName(const PinInfo pin);

/**
 * @brief Returns the functional family of a pin (UART, SPI, GPIO, etc.).
 *
 * @param pin  PinInfo structure containing pin metadata.
 * @return Pointer to a string describing the pin family.
 */
const char* pinFamily(const PinInfo pin);

/**
 * @brief Checks whether a given pin belongs to the GPIO family.
 *
 * @param pin  PinInfo structure containing pin metadata.
 * @return true if the pin is a valid GPIO pin, false otherwise.
 */
bool isValidGPIO(const PinInfo& pin);

/**
 * @brief Checks whether a given pin belongs to the PWM family.
 *
 * @param pin  PinInfo structure containing pin metadata.
 * @return true if the pin is a valid PWM pin, false otherwise.
 */
bool isValidPWM(const PinInfo& pin);

/**
 * @brief Checks whether a given pin belongs to the ANALOG family.
 *
 * @param pin  PinInfo structure containing pin metadata.
 * @return true if the pin is a valid ANALOG pin, false otherwise.
 */
bool isValidAnalog(const PinInfo& pin);


/* -------------------------------------------------------------------------- */
/*                               Diagnostics                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Diagnoses all ANALOG pins.
 *
 * Configures each analog pin as INPUT and measures whether voltage is present.
 * Also calculates and prints the approximate percentage of the input voltage.
 */
void diagnoseAnalog();

/**
 * @brief Diagnoses all GPIO pins.
 *
 * Configures each GPIO pin as INPUT and checks whether a HIGH or LOW signal
 * is detected.
 */
void diagnoseGPIO();

/**
 * @brief Diagnoses all PWM pins.
 *
 * Configures each PWM pin as INPUT and checks whether voltage is present.
 */
void diagnosePWM();

/**
 * @brief Runs all pin diagnostics and prints the results via Serial.
 *
 * Executes analog, GPIO, and PWM diagnostics sequentially and outputs a
 * complete report.
 */
void fullDiagnosticsPins();



#endif  // PINS_H
