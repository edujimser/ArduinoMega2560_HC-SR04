#ifndef MESSAGE_RELEASE_H
#define MESSAGE_RELEASE_H

#include <Arduino.h>
#include "system/config/config.h"

/**
 * @brief Prints an enriched log message to the serial monitor.
 *
 * This function should be defined in the corresponding source (.cpp) file.
 * It generates a structured output that includes:
 * - Compilation date and time
 * - Source file from which it is called
 * - Name of the calling function
 * - Custom message
 *
 * Output format:
 * [date time] file::function ➤ message
 *
 * Example:
 * [Aug 7 2025 01:23:00] main.cpp::setup ➤ System initialization...
 *
 * @param message   Custom text to be printed
 * @param file      Source file name (__FILE__)
 * @param function  Name of the calling function (__FUNCTION__)
 * @param date      Compilation date (__DATE__)
 * @param time      Compilation time (__TIME__)
 */
void standardMessage(const char* message,
                     const char* file,
                     const char* function,
                     const char* date,
                     const char* time);


/**
 * Prints a centered header with decorative lines above/below.
 * This is a reduced version without metadata, ideal for section titles.
 *
 * @param message   The title text to display centered.
 * @param width     Total width of the decorative line (default: 200).
 * @param deco      Decorative character to use (default: '-').
 */
void standardHeader(const char* message,
                    uint16_t width = 200,
                    char deco = '-');


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
 * @param date      Compilation date (recommended: __DATE__).
 * @param time      Compilation time (recommended: __TIME__).
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
                          uint16_t width = 120,
                          char deco = '-');


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
void standardErrorMessage(const char* message, const char* file, const char* function, const char* date, const char* time, int line);


/**
 * @brief Displays the current system configuration status on the serial monitor.
 * 
 * This function prints the current values of the fields defined in the 
 * configuracionMain structure, such as debug mode and LED project status, 
 * in a user-friendly format to facilitate system diagnostics or verification.
 * 
 * @param configuration  Structure containing the main system configuration.
 */
void showConfigurationMessage(const configuracionMain& configuration);


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
                  const char* appDate);

#endif // MESSAGE_RELEASE_H
