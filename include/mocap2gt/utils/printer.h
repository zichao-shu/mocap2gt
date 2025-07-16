// Copyright 2025 Anonymous Authors 
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

namespace mocap2gt {

/**
 * @brief Utility class that allows for various levels of printing.
 */
class Printer {
 public:
  /**
   * @brief Enumerations of different print levels.
   *
   * - PrintLevel::ALL     : All PRINT(XXXX) will output to the console.
   * - PrintLevel::DEBUG   : "DEBUG", "INFO", "WARNING" and "ERROR" will be
   * printed. "ALL" will be silenced.
   * - PrintLevel::INFO    : "INFO", "WARNING" and "ERROR" will be printed.
   * "ALL" and "DEBUG" will be silenced.
   * - PrintLevel::WARNING : "WARNING" and "ERROR" will be printed. "ALL",
   * "DEBUG" and "INFO" will be silenced.
   * - PrintLevel::ERROR   : Only "ERROR" will be printed. All the rest are
   * silenced.
   * - PrintLevel::SILENT  : All PRINT(XXXX) will be silenced.
   */
  enum PrintLevel {
    ALL = 0,
    DEBUG = 1,
    INFO = 2,
    WARNING = 3,
    ERROR = 4,
    SILENT = 5
  };

  /**
   * @brief Set the print level to control the output content.
   * @param level Target level.
   *
   * One can do the following: Printer::SetPrintLevel("INFO").
   */
  static void SetPrintLevel(const std::string &level);

  /**
   * @brief Set the print level to control the output content.
   * @param level Target level.
   *
   * One can do the following:
   * Printer::SetPrintLevel(Printer::PrintLevel::INFO).
   */
  static void SetPrintLevel(PrintLevel level);

  /**
   * @brief The print function that allows for various levels of printing.
   * @param level The print level for this print call.
   * @param text_color The color and style for this print call.
   * @param location The location the print was made from.
   * @param line The line the print was made from.
   * @param format The printf format.
   */
  static void PrintCall(PrintLevel level, const char *text_color,
                        const char location[], const char line[],
                        const char *format, ...);

  // The current print level.
  static PrintLevel current_print_level_;

 private:
  // The max length for the file path, to avoid very long file paths from.
  static constexpr uint32_t max_file_path_length_ = 30;
};

}  // namespace mocap2gt

// ANSI escape codes for text color and style formatting.
#define RESET "\033[0m"
#define BLACK "\033[30m"                 // Black
#define RED "\033[31m"                   // Red
#define GREEN "\033[32m"                 // Green
#define YELLOW "\033[33m"                // Yellow
#define BLUE "\033[34m"                  // Blue
#define MAGENTA "\033[35m"               // Magenta
#define CYAN "\033[36m"                  // Cyan
#define WHITE "\033[37m"                 // White
#define REDPURPLE "\033[95m"             // Red Purple
#define BOLDBLACK "\033[1m\033[30m"      // Bold Black
#define BOLDRED "\033[1m\033[31m"        // Bold Red
#define BOLDGREEN "\033[1m\033[32m"      // Bold Green
#define BOLDYELLOW "\033[1m\033[33m"     // Bold Yellow
#define BOLDBLUE "\033[1m\033[34m"       // Bold Blue
#define BOLDMAGENTA "\033[1m\033[35m"    // Bold Magenta
#define BOLDCYAN "\033[1m\033[36m"       // Bold Cyan
#define BOLDWHITE "\033[1m\033[37m"      // Bold White
#define BOLDREDPURPLE "\033[1m\033[95m"  // Bold Red Purple

// Converts anything to a string.
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// The print function with different levels.
#define PRINT_ALL(x...)                                                   \
  mocap2gt::Printer::PrintCall(mocap2gt::Printer::PrintLevel::ALL, WHITE, \
                               __FILE__, TOSTRING(__LINE__), x);
#define PRINT_DEBUG(x...)                                                   \
  mocap2gt::Printer::PrintCall(mocap2gt::Printer::PrintLevel::DEBUG, WHITE, \
                               __FILE__, TOSTRING(__LINE__), x);
#define PRINT_INFO(x...)                                                  \
  mocap2gt::Printer::PrintCall(mocap2gt::Printer::PrintLevel::INFO, BLUE, \
                               __FILE__, TOSTRING(__LINE__), x);
#define PRINT_WARNING(x...)                                                 \
  mocap2gt::Printer::PrintCall(mocap2gt::Printer::PrintLevel::WARNING,      \
                               BOLDREDPURPLE, __FILE__, TOSTRING(__LINE__), \
                               x);
#define PRINT_ERROR(x...)                                                     \
  mocap2gt::Printer::PrintCall(mocap2gt::Printer::PrintLevel::ERROR, BOLDRED, \
                               __FILE__, TOSTRING(__LINE__), x);
