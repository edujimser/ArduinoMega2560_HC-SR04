# ArduinoMega2560

This repository contains code templates, configurations, and documentation designed for projects based on the Arduino Mega 2560 microcontroller (ATmega2560).  
It serves as a starting point or foundation for developers who want to build modular, optimized, and well‑structured applications on the Mega2560 platform.

The goal of this repository is to provide a ready‑to‑use base project that simplifies the setup of new developments.  
Instead of starting from scratch, you can clone this repository and immediately have:
- A clean project structure compatible with PlatformIO and Arduino IDE.
- Preconfigured build flags for memory optimization and performance.
- Example modules for diagnostics, GPIO, PWM, UART, and EEPROM.
- Documentation templates (README, diagrams, configuration notes) to keep your project organized.

---

## Table of Contents

- [Introduction](#introduction)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Debug](#debug)
- [Functions](#functions)
- [Arduino Mega 2560 Pinout](#arduino-mega-2560-pinout)
- [PlatformIO Configuration](#platformio-configuration)
- [HC-SR04 Ultrasonic Sensor Library](#hc-sr04-ultrasonic-sensor-library)
- [License](#license)

---

## Introduction

This project serves as a foundation for various Arduino Mega 2560-based applications. It leverages the PlatformIO IDE for streamlined development, building, and deployment.

## Getting Started

These instructions will guide you through setting up the project on your local machine for development and deployment.

### Prerequisites

Before you begin, ensure you have the following installed:

- [PlatformIO IDE](https://platformio.org/platformio-ide)

### Installation

```bash
git clone https://github.com/edujimser/ArduinoMega2560.git
cd ArduinoMega2560
```

Open the project in PlatformIO IDE and build/upload as usual.

---

## Project Structure

```
ArduinoMega2560/
├── .gitignore
├── .vscode/
├── include/
├── lib/
│   ├── avr-debugger/
│   └── hc-sr04/          # HC-SR04 ultrasonic sensor library
├── platformio.ini
├── src/
│   ├── main.cpp
│   └── system/
└── README.md
```

---

## Usage

Basic example in `src/main.cpp`:

```cpp
#include <Arduino.h>
#include "hc-sr04/hc-sr04.h"

static PinInfo Trig = Pins::GPIO[29]; // Pin 50
static PinInfo Echo = Pins::GPIO[30]; // Pin 51
static HCSR04 sonar(Trig, Echo, 20);  // Max distance 20 cm

void setup() {
  Serial.begin(9600);
}

void loop() {
  sonar.printStateEcho(sonar.ping_cm());
  delay(500);
}
```

---

## Debug

This project includes a debugging system for the Arduino Mega 2560 using **avr-stub**, **GDB**, and an **FT232BL** USB–Serial adapter. It enables advanced firmware debugging on a microcontroller that does not support hardware debugging natively.

### Requirements for debug mode

- Arduino Mega 2560  
- FT232BL USB–Serial adapter  
- PlatformIO (VSCode)  
- `avr-stub` library (included in project)  
- Wiring between FT232BL and Mega as described below

### FT232BL Wiring

| FT232BL | Mega 2560 |
|---------|-----------|
| TXD     | RX0 (0)   |
| RXD     | TX0 (1)   |
| GND     | GND       |

### PlatformIO debug environment example

```ini
[env:debug]
build_type = debug
debug_tool = custom
debug_port = /dev/ttyUSB0

build_flags =
    -DDEBUG_MODE
    -Og
    -g

lib_deps =
    jdolinay/avr-debugger
```

---

## Functions

- `setup()`: Called once at startup. Initialize pins, serial, peripherals.  
- `loop()`: Main program loop. Read sensors, control actuators, handle logic.  
- Helper modules and utilities are provided under `src/system/`.

---

## Arduino Mega 2560 Pinout

- **Digital Pins:** 0–53 (some with special functions)  
- **Analog Inputs:** A0–A15  
- **PWM Pins:** Marked with `~`  
- **Serial Ports:** Multiple UARTs (Serial, Serial1, Serial2, Serial3)  
- **SPI / I2C:** Dedicated pins for SPI and I2C communication  
- **AREF / GND / VCC:** Power and reference pins

---

## PlatformIO Configuration

The `platformio.ini` file configures the build environment. Key points:

- `platform = atmelavr` and `board = megaatmega2560`  
- `framework = arduino`  
- Example build flags include `-std=gnu++17`, optimization flags, and debug macros  
- Debugging support via `avr-stub` is included as an optional environment

Example snippet:

```ini
[env:megaatmega2560]
platform = atmelavr
board = megaatmega2560
framework = arduino
upload_port = COM3
monitor_port = COM3

build_unflags = 
    -std=gnu++11

build_flags = 
    -std=gnu++17
    -DAVR8_UART_NUMBER=3
    -DDEBUG_MODE=0
    -Os
    -flto
    -fno-exceptions

debug_tool = avr-stub
debug_port = COM6
monitor_speed = 57600
```

---

## HC-SR04 Ultrasonic Sensor Library

This repository includes a custom library to control the **HC-SR04 ultrasonic distance sensor** with the Arduino Mega 2560. It uses direct port manipulation for optimized performance and provides clear state handling and range validation.

### Library location

```
lib/hc-sr04/
├── hc-sr04.h   # Header with class definition and constants
└── hc-sr04.cpp # Implementation
```

### Public API summary

- `HCSR04(PinInfo PinTrigger, PinInfo PinEcho, unsigned int maxDistanceCm = HC_SR04_DEFAULT_MAX_DISTANCE_CM)`  
  Constructor: configure trigger/echo pins and maximum distance.

- `unsigned long ping()`  
  Returns measured echo time in microseconds (µs). Returns 0 on failure/timeout.

- `unsigned long ping_cm()`  
  Returns distance in centimeters (cm). Uses conversion factor `HC_SR04_CONVERSION_US_CM`.

- `boolean pingTrigger()`  
  Internal trigger/echo capture routine. Returns `true` on successful measurement.

- `void printStateEcho(unsigned long distance)`  
  Prints state and distance to Serial.

### Constants used by the library

```cpp
#define HC_SR04_DEFAULT_MAX_DISTANCE_CM 50
#define HC_SR04_CONVERSION_US_CM 58
#define HC_SR04_DELAY_TRIGGER_MS 12
#define HC_SR04_FAIL_CYCLES_LIMIT 20
```

### Features

- Direct port manipulation for faster I/O on Mega2560.  
- State machine for echo detection with distinct states: `ECHO_OK`, `ECHO_TIMEOUT_UP`, `ECHO_TIMEOUT_DOWN`, `ECHO_OK_OUT_LIMIT_CM`, `ECHO_NOOK`.  
- Timeout calculation based on configured maximum distance:
  ```cpp
  maxEchotimeUs = maxDistanceCm * HC_SR04_CONVERSION_US_CM;
  ```
- Range validation: if computed distance > `maxDistanceCm`, library sets `ECHO_OK_OUT_LIMIT_CM`.

### Example usage

```cpp
#include "hc-sr04/hc-sr04.h"

static PinInfo Trig = Pins::GPIO[29]; // Pin 50
static PinInfo Echo = Pins::GPIO[30]; // Pin 51
static HCSR04 sonar(Trig, Echo, 100); // Max distance 100 cm

void setup() {
  Serial.begin(9600);
}

void loop() {
  unsigned long distance = sonar.ping_cm();
  sonar.printStateEcho(distance);
  delay(500);
}
```

### Debugging tips

- Print `flagEchoTime` (µs) before conversion to verify raw timing values.  
- Use `pulseIn()` in a simple test sketch to confirm hardware wiring and sensor behavior before using direct port code.  
- Ensure trigger pin is driven low for a short time before sending the 10–12 µs HIGH pulse.  
- Keep sensor and wiring away from noisy power sources; add decoupling capacitors if needed.

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
