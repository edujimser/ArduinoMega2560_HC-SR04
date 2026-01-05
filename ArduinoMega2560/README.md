

# ArduinoMega2560

This repository contains code templates, configurations, and documentation designed for projects based on the Arduino Mega 2560 microcontroller (ATmega2560).  
It serves as a starting point or foundation for developers who want to build modular, optimized, and well‑structured applications on the Mega2560 platform.

The goal of this repository is to provide a ready‑to‑use base project that simplifies the setup of new developments.  
Instead of starting from scratch, you can clone this repository and immediately have:
- A clean project structure compatible with PlatformIO and Arduino IDE.
- Preconfigured build flags for memory optimization and performance.
- Example modules for diagnostics, GPIO, PWM, UART, and EEPROM.
- Documentation templates (README, diagrams, configuration notes) to keep your project organized.




## Table of Contents

- [Introduction](#introduction)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Debug](#Debug)
- [Functions](#functions)
- [Arduino Mega 2560 Pinout](#arduino-mega-2560-pinout)
- [PlatformIO Configuration](#platformio-configuration)
- [License](#license)

## Introduction

This project serves as a foundation for various Arduino Mega 2560-based applications. It leverages the PlatformIO IDE for streamlined development, building, and deployment.

## Getting Started

These instructions will guide you through setting up the project on your local machine for development and deployment.

### Prerequisites

Before you begin, ensure you have the following installed:

-   [PlatformIO IDE](https://platformio.org/platformio-ide) (Installation guide available on the PlatformIO website)

### Installation

1.  **Clone the repository:**

    ```bash
    git clone https://github.com/edujimser/ArduinoMega2560.git
    cd ArduinoMega2560
    ```

2.  **Open the project in PlatformIO IDE:**

    -   Launch PlatformIO IDE.
    -   Click "Open Project" and select the cloned `ArduinoMega2560` directory.

3.  **Build the project:**

    -   In the PlatformIO IDE, click the "PlatformIO" icon in the bottom toolbar.
    -   Select "Build" to compile the code.

4.  **Upload the code to your Arduino Mega 2560:**

    -   Connect your Arduino Mega 2560 to your computer via USB.
    -   In the PlatformIO IDE, select "Upload" to flash the compiled code to your board.

## Project Structure

The project structure is organized as follows:

```
ArduinoMega2560/
├── .gitignore              # Specifies intentionally untracked files that Git should ignore
├── .vscode/                # Contains VS Code specific settings
├── include/                # Header files for the project
├── lib/                    # Libraries used in the project
│   └── avr-debugger/       # Example library (if applicable)
├── platformio.ini          # PlatformIO configuration file
├── src/                    # Source code files
│   ├── main.cpp            # Main application file
│   └── system/             # System-related source files (if applicable)
└── README.md             # Project documentation
```

## Usage

The `src/main.cpp` file contains the main application logic. You can modify this file to implement your desired functionality.  Refer to the comments within `main.cpp` for guidance on how to use the code.

Example (from `src/main.cpp`):

```cpp
#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello, Arduino!");
  delay(1000);
}
```

## Debug
This project includes a full debugging system for the Arduino Mega 2560 using **avr-stub**, **GDB**, and an **FT232BL** USB–Serial adapter.  
This enables professional-level firmware debugging on a microcontroller that does not support hardware debugging natively.

---

###  Requirements

To use the debug mode, you will need:

- Arduino **Mega 2560**
- **FT232BL** USB–Serial adapter
- **PlatformIO** (VSCode)
- The `avr-stub` library (already included in the project)
- Basic wiring between the FT232BL and the Mega

---

###  FT232BL Wiring

Connect the FT232BL to the Mega 2560 as follows:

| FT232BL | Mega 2560 |
|---------|-----------|
| TXD     | RX0 (0)   |
| RXD     | TX0 (1)   |
| GND     | GND       |

> The FT232BL is used by GDB to communicate with the microcontroller.

---

###  PlatformIO Configuration

The debug environment is already defined in `platformio.ini`:

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

## Functions

Here's a description of the key functions used in the `src/main.cpp` file:

*   **`void setup()`**: This function is called once at the beginning of the program. It is typically used to initialize variables, set pin modes, start serial communication, and perform other setup tasks.

    *   `Serial.begin(9600)`: Initializes serial communication at a baud rate of 9600. This allows the Arduino to send data to the serial monitor for debugging and monitoring.

*   **`void loop()`**: This function is called repeatedly in an infinite loop. It contains the main logic of the program, such as reading sensor data, controlling actuators, and responding to user input.

    *   `Serial.println("Hello, Arduino!")`: Prints the text "Hello, Arduino!" to the serial monitor.
    *   `delay(1000)`: Pauses the program for 1000 milliseconds (1 second).

## Arduino Mega 2560 Pinout

The Arduino Mega 2560 has a variety of pins that can be used for different purposes:

*   **Digital Pins (0-53):** These pins can be configured as either inputs or outputs. Some digital pins have special functions, such as PWM (Pulse Width Modulation) for controlling the brightness of LEDs or the speed of motors.
*   **Analog Input Pins (A0-A15):** These pins can read analog signals from sensors, such as potentiometers or temperature sensors.
*   **PWM Pins:** Marked with a "~" symbol, these pins can be used for PWM output.
*   **Serial Communication Pins (TX, RX):** Used for serial communication with other devices.
*   **SPI Pins:** Used for SPI (Serial Peripheral Interface) communication.
*   **I2C Pins (SDA, SCL):** Used for I2C (Inter-Integrated Circuit) communication.
*   **AREF:** Analog Reference pin for analog inputs.
*   **GND:** Ground pins.

## PlatformIO Configuration

The `platformio.ini` file configures the PlatformIO build environment for this project. Here's a breakdown of the key settings:

```ini
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed, and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit the documentation for other options and examples:
; https://docs.platformio.org/page/projectconf.html

;----------------------------------------------------------------------------------------------------------------------------------------------------------------
;------ Project Configuration ------
[env:megaatmega2560]
platform = atmelavr           ; Specifies the development platform (Atmel AVR)
board = megaatmega2560       ; Specifies the board (Arduino Mega 2560)
framework = arduino            ; Specifies the framework (Arduino)
upload_port = COM3             ; Port where your board is connected (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux/Mac)
monitor_port = COM3          ; Must match the physical port of your board (same as debug_port normally)
                             ; To check the port in CMD: connect and disconnect the board, then run `mode` in CMD
;----------------------------------------------------------------------------------------------------------------------------------------------------------------
;------ Standard dependencies and extra configurations ------
; Adapting the C++ version for the project: removing GNU++11 standard to insert GNU++17
; https://docs.platformio.org/en/latest/projectconf/advanced_scripting.html#build-unflags-and-build-flags
build_unflags = 
    -std=gnu++11              ; Remove the C++11 standard to avoid conflicts with the C++17 standard
build_flags = 
    -std=gnu++17              ; Use the C++17 standard with GNU extensions (more flexible than the pure standard)
    -DAVR8_UART_NUMBER=3      ; Define a macro to indicate that UART number 3 will be used on AVR microcontrollers
    -DEBUG_MODE=0             ; Activate debug mode to include debugging code
    -Os                       ; Change the optimization level here:
                                ; -O0  → no optimization (debugging)
                                ; -O1  → light optimization
                                ; -O2  → balanced optimization
                                ; -O3  → maximum speed
                                ; -Os  → optimize for size
    -flto                     ; Enable Link Time Optimization (LTO) for better optimization across files
    -fno-exceptions           ; Disable exceptions to reduce code size and improve performance

;----------------------------------------------------------------------------------------------------------------------------------------------------------------
;------ Artificial Debugging Dependencies ------
; Notes:
;   - Configure build_flags (DAVR8_UART_NUMBER, DEBUG_MODE) to activate debug mode
; Activate simulated debugging system with avr-stub
debug_tool = avr-stub         ; Uses the 'avr-debugger' library to simulate debugging on AVR chips
; Virtual serial port for debugging
debug_port = COM6             ; Port where your board is connected (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux/Mac)
                             ; To check the port in CMD: connect and disconnect the board, then run `mode` in CMD
; Serial monitor communication speed
monitor_speed = 57600         ; Baud rate. Must match the one used in Serial.begin() in your code

;----------------------------------------------------------------------------------------------------------------------------------------------------------------
;------ RFID-RC522 Dependencies ------
; The following line tells PlatformIO to automatically download and include 
; the MFRC522 library by Miguel Balboa from PlatformIO's official registry.
; This library allows controlling the RFID RC522 reader module for reading/writing tags and cards.
; lib_deps = /lib/Ard - opcional descomentar si se quiere usar la librería de Arduino por descarga

[platformio]
description = Sensors
```

### Explanation of Settings:

*   **`[env:megaatmega2560]`**: This section defines the environment for the Arduino Mega 2560 board.  `env` stands for environment. You can have multiple environments for different boards or configurations.

    *   `platform = atmelavr`: Specifies that the Atmel AVR platform will be used.
    *   `board = megaatmega2560`:  Specifies that the target board is the Arduino Mega 2560.
    *   `framework = arduino`: Specifies that the Arduino framework will be used.
    *   `upload_port = COM3`: Defines the serial port used for uploading the compiled code to the board.  **Important:**  You need to change this to the correct port for your system (e.g., `/dev/ttyUSB0` on Linux, `COM3` or similar on Windows).
    *   `monitor_port = COM3`: Defines the serial port used for the serial monitor. This should typically match the `upload_port`.

*   **`build_unflags`**: These settings remove default compiler flags.

    *   `-std=gnu++11`:  Removes the default C++11 standard flag. This is done to allow the use of C++17.

*   **`build_flags`**: These settings add custom compiler flags.

    *   `-std=gnu++17`:  Specifies that the C++17 standard should be used for compilation. C++17 provides more modern language features.
    *   `-DAVR8_UART_NUMBER=3`: Defines a macro that might be used in the code to specify

    ### License
    
    This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
    
    ```
    MIT License
    
    Copyright (c) [Año] [Nombre del titular del copyright]
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
    ```