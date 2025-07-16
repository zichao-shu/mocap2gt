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

#include "mocap2gt/utils/printer.h"

namespace mocap2gt {

// Define the static variable.
Printer::PrintLevel Printer::current_print_level_ = PrintLevel::INFO;

void Printer::SetPrintLevel(const std::string &level) {
  if (level == "ALL") {
    SetPrintLevel(PrintLevel::ALL);
  } else if (level == "DEBUG") {
    SetPrintLevel(PrintLevel::DEBUG);
  } else if (level == "INFO") {
    SetPrintLevel(PrintLevel::INFO);
  } else if (level == "WARNING") {
    SetPrintLevel(PrintLevel::WARNING);
  } else if (level == "ERROR") {
    SetPrintLevel(PrintLevel::ERROR);
  } else if (level == "SILENT") {
    SetPrintLevel(PrintLevel::SILENT);
  } else {
    std::cout << "Invalid print level requested: " << level << std::endl;
    std::cout << "Valid levels are: ALL, DEBUG, INFO, WARNING, ERROR, SILENT"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

void Printer::SetPrintLevel(PrintLevel level) {
  Printer::current_print_level_ = level;
  std::cout << "Setting printing level to: ";
  switch (current_print_level_) {
    case PrintLevel::ALL:
      std::cout << "ALL";
      break;
    case PrintLevel::DEBUG:
      std::cout << "DEBUG";
      break;
    case PrintLevel::INFO:
      std::cout << "INFO";
      break;
    case PrintLevel::WARNING:
      std::cout << "WARNING";
      break;
    case PrintLevel::ERROR:
      std::cout << "ERROR";
      break;
    case PrintLevel::SILENT:
      std::cout << "SILENT";
      break;
    default:
      std::cout << std::endl;
      std::cout << "Invalid print level requested: " << level << std::endl;
      std::cout << "Valid levels are: ALL, DEBUG, INFO, WARNING, ERROR, SILENT"
                << std::endl;
      std::exit(EXIT_FAILURE);
  }
  std::cout << std::endl;
}

void Printer::PrintCall(PrintLevel level, const char *text_color,
                        const char location[], const char line[],
                        const char *format, ...) {
  // Only print for the current level.
  if (static_cast<int>(level) <
      static_cast<int>(Printer::current_print_level_)) {
    return;
  }

  // Set text color using the ANSI escape code.
  printf("%s", text_color);

  // Print the location for debug output, and truncate the filename to the max
  // size for the filepath.
  if (static_cast<int>(Printer::current_print_level_) <=
      static_cast<int>(Printer::PrintLevel::DEBUG)) {
    std::string path(location);
    std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
    if (base_filename.size() > max_file_path_length_) {
      printf("%s", base_filename
                       .substr(base_filename.size() - max_file_path_length_,
                               base_filename.size())
                       .c_str());
    } else {
      printf("%s", base_filename.c_str());
    }
    printf(":%-3s ", line);
  }

  // Print the rest of the args.
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);

  // Reset text color to default.
  printf("\033[0m");
}

}  // namespace mocap2gt
