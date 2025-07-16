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

int main() {
  // Set the print level to ALL.
  mocap2gt::Printer::SetPrintLevel(mocap2gt::Printer::PrintLevel::ALL);

  // Test different levels of output.
  PRINT_ERROR("This is an ERROR message, the font should be BOLD RED. \n")
  PRINT_WARNING(
      "This is a WARNING message, the font should be BOLD RED PURPLE. \n")
  PRINT_INFO("This is an INFO message, the font should be BLUE. \n")
  PRINT_DEBUG("This is a DEBUG message, the font should be WHITE. \n")
  PRINT_ALL("This is an ALL message, the font should be WHITE. \n")

  return 0;
}
