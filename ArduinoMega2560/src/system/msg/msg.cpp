#include "system/msg/msg.h"

/**
 * Prints a standardized, nicely formatted message to the Serial Monitor.
 * The message will be centered between two decorative lines.
 *
 * @param message   The main message to display.
 * @param file      Source file name where the message originated.
 * @param function  Function name from which this is called.
 * @param date      Compilation date or custom date string.
 * @param time      Compilation time or custom time string.
 */
void standardMessage(const char* message, 
                     const char* file, 
                     const char* function, 
                     const char* date, 
                     const char* time) {
    const uint8_t totalWidth = 200; // Total width of the decorative line
    int msgLen = strlen(message);   // Length of the message text
    int padding = (totalWidth - msgLen) / 2; // Spaces to add on the left to center the text

    Serial.println();

    // ─────────────────────────────
    // Top decorative line
    // ─────────────────────────────
    for (uint8_t i = 0; i < totalWidth; i++) Serial.write('-');
    Serial.println();

    // ─────────────────────────────
    // Centered message text
    // ─────────────────────────────
    for (int i = 0; i < padding; i++) Serial.write(' '); // Left padding
    Serial.println(message); // Main message

    // ─────────────────────────────
    // Bottom decorative line
    // ─────────────────────────────
    for (uint8_t i = 0; i < totalWidth; i++) Serial.write('-');
    Serial.println();

    // ─────────────────────────────
    // Detailed information below the main message
    // ─────────────────────────────
    Serial.print("[");
    Serial.print(date);
    Serial.print(" ");
    Serial.print(time);
    Serial.print("] ");
    Serial.print(file);
    Serial.print("::");
    Serial.print(function);
    Serial.print(" ➤ ");
    Serial.println(message);

    Serial.println();
}


/**
 * Prints a decorative header with the given message centered.
 *
 * @param message The text to be displayed centered.
 * @param width   The total width of the header line.
 * @param deco    The decorative character to use for the top/bottom lines.
 */
void standardHeader(const char* message,
                    uint16_t width,
                    char deco) {
    int msgLen = strlen(message);
    int totalDeco = width - msgLen - 2; // Subtract 2 for spacing around the message
    if (totalDeco < 0) totalDeco = 0;

    int leftDeco = totalDeco / 2;
    int rightDeco = totalDeco - leftDeco;

    // 🔹 Blank line before the header
    Serial.println();

    // 🔹 Left decorative padding
    for (int i = 0; i < leftDeco; i++) Serial.write(deco);

    // 🔹 Centered message with spacing
    Serial.print(' ');
    Serial.print(message);
    Serial.print(' ');

    // 🔹 Right decorative padding
    for (int i = 0; i < rightDeco; i++) Serial.write(deco);

    // 🔹 Blank line after the header
    Serial.println();
    Serial.println();
}


/**
 * @brief Prints a decorated header with centered text and padding on both sides.
 *
 * This function builds a single line consisting of a combined text
 * that includes the message, file, function, date, and time, surrounded
 * by decorative characters on the left and right to achieve a total line width
 * specified by the user.
 *
 * Output format example:
 *  ----- Message | file | function | date time -----
 *
 * @param message   Main text or description to display.
 * @param file      File path or source file name.
 * @param function  Name of the function where this is called.
 * @param date      Compilation date (recommended: __ time      CompilationDATE__).
 * @param time (recommended: __TIME__).
 * @param width     Total width of the line (default: 120).
 * @param deco      Decorative character (default: '-').
 *
 * @note The number of decorative characters is automatically adjusted
 *       to keep the text centered within the given width.
 */
void standardHeaderFull(const char* message,
                        const char* file,
                        const char* function,
                        const char* date,
                        const char* time,
                        uint16_t width,
                        char deco) {
    char buffer[256];
    // Build the full message with metadata
    snprintf(buffer, sizeof(buffer), "%s | %s | %s | %s %s",
             message, file, function, date, time);

    int msgLen = strlen(buffer);
    int totalDeco = width - msgLen - 2; // Subtract 2 for spacing around the message
    if (totalDeco < 0) totalDeco = 0;   // Prevent negative padding

    int leftDeco = totalDeco / 2;
    int rightDeco = totalDeco - leftDeco;

    // 🔹 Blank line before the header
    Serial.println();

    // 🔹 Left decorative padding
    for (int i = 0; i < leftDeco; i++) Serial.write(deco);

    // 🔹 Centered message with spacing
    Serial.print(' ');
    Serial.print(buffer);
    Serial.print(' ');

    // 🔹 Right decorative padding
    for (int i = 0; i < rightDeco; i++) Serial.write(deco);

    // 🔹 Blank lines after the header
    Serial.println();
    Serial.println();
}


/**
 * @brief Prints an enriched error message to the serial monitor, including contextual information.
 *
 * This function generates a structured output that helps identify the origin of runtime errors.
 * The output includes:
 * - Compilation date and time
 * - Source file name
 * - Function name from which it is invoked
 * - Line number where the function was called
 * - Custom error message
 *
 * Output format:
 * [date time] file::function (Line N) ❌ ERROR ➤ message
 *
 * Example output:
 * [Aug 7 2025 01:23:00] sensor.cpp::readSensor (Line 42) ❌ ERROR ➤ Failed to initialize sensor
 *
 * @param message   Description of the specific error
 * @param file      Source file name (__FILE__)
 * @param function  Function name where the error was detected (__FUNCTION__)
 * @param date      Compilation date (__DATE__)
 * @param time      Compilation time (__TIME__)
 * @param line      Line number in the file where the function is invoked (__LINE__)
 */
void standardErrorMessage(const char* message, const char* file, const char* function, const char* date, const char* time, int line) {
  Serial.println();
  Serial.print("[");
  Serial.print(date);
  Serial.print(" ");
  Serial.print(time);
  Serial.print("] ");
  Serial.print(file);
  Serial.print("::");
  Serial.print(function);
  Serial.print(" (Line ");
  Serial.print(line);
  Serial.print(") ❌ ERROR ➤ ");
  Serial.println(message);
  Serial.println();
}


/**
 * @brief Prints the current system configuration status to the serial monitor.
 * 
 * This function displays the values of the fields contained in the 
 * configuracionMain structure, such as debug mode and LED project status,
 * using a clear and friendly visual format for easy reading on the serial monitor.
 * It includes separators and informative emojis to enhance presentation.
 * 
 * @param configuration  Reference to the current system configuration structure.
 */
void showConfigurationMessage(const configuracionMain& configuration) {
  standardMessage("Current System Configuration",
                     __FILE__,
                     __FUNCTION__,
                     __DATE__,
                     __TIME__);
  Serial.print("🔧 Debug mode: ");
  Serial.println(configuration.debugMode ? "Enabled" : "Disabled");

  standardHeaderFull("End of Configuration",
                     __FILE__,
                     __FUNCTION__,
                     __DATE__,
                     __TIME__,
                     120,
                     '-');

  
}


void showMsgPrinLine(char c, int length) {
   for (int i = 0; i < length; i++) {
        Serial.print(c);
    }
    Serial.println();
  }


/**
 * @brief Prints firmware version details to the serial monitor.
 * 
 * Displays firmware and application information (version, name, date, author) 
 * defined by macros in a clear format for easy verification.
 */
void printVersion(const char* fwVersion,
                  const char* fwName,
                  const char* fwDate,
                  const char* fwAuthor,
                  const char* appVersion,
                  const char* appName,
                  const char* appDate) {
    standardMessage("Firmware Version Information",
                     __FILE__,
                     __FUNCTION__,
                     __DATE__,
                     __TIME__);
    Serial.print("Firmware Version: "); Serial.println(fwVersion);
    Serial.print("Firmware Name: ");    Serial.println(fwName);
    Serial.print("Firmware Date: ");    Serial.println(fwDate);
    Serial.print("Firmware Author: ");  Serial.println(fwAuthor);
    Serial.print("App Version: ");      Serial.println(appVersion);
    Serial.print("App Name: ");         Serial.println(appName);
    Serial.print("App Date: ");         Serial.println(appDate);
    }