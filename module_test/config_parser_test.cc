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

#include "mocap2gt/utils/config_parser.h"

int main() {
  // Path to the config file.
  std::string config_file_path =
      "../../resource/estimator_test_data/estimator_config.yaml";

  // Test the config parser.
  std::shared_ptr<mocap2gt::ConfigParser> config_parser(
      new mocap2gt::ConfigParser(config_file_path));

  return 0;
}
