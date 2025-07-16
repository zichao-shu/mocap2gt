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

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "mocap2gt/sensor_meas/imu_meas_manager.h"
#include "mocap2gt/sensor_meas/mocap_meas_manager_base.h"
#include "mocap2gt/utils/printer.h"

namespace mocap2gt {

class DataReader {
 public:
  /**
   * @brief Read the IMU data and feed them to the IMU measurement manager.
   *
   * @param imu_manager IMU measurement manager.
   * @param imu_data_path IMU data path.
   */
  static void ReadImuData(std::shared_ptr<mocap2gt::ImuMeasManager> imu_manager,
                          const std::string &imu_data_path) {
    std::ifstream file(imu_data_path);
    if (!file.is_open()) {
      PRINT_ERROR("[Error]: Failed to open imu data file: %s \n",
                  imu_data_path.c_str());
      std::exit(EXIT_FAILURE);
    }

    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      std::vector<double> values;

      try {
        while (std::getline(ss, value, ',')) {
          values.push_back(std::stod(value));
        }
      } catch (const std::exception &e) {
        PRINT_ERROR(
            "[Error]: Each line in the IMU file must be in format "
            "\"timestamp(ns),wx(rad/s),wy(rad/s),wz(rad/s),"
            "ax(m/s^2),ay(m/s^2),az(m/s^2)\" \n")
        std::exit(EXIT_FAILURE);
      }

      if (values.size() == 7) {
        values[0] /= 1e9;
      } else {
        PRINT_ERROR(
            "[Error]: Each line in the IMU file must be in format "
            "\"timestamp(ns),wx(rad/s),wy(rad/s),wz(rad/s),"
            "ax(m/s^2),ay(m/s^2),az(m/s^2)\" \n")
        std::exit(EXIT_FAILURE);
      }

      double timestamp(values[0]);
      Eigen::Vector3d gyr_m(values[1], values[2], values[3]);
      Eigen::Vector3d acc_m(values[4], values[5], values[6]);

      if (!imu_manager->FeedImu(timestamp, acc_m, gyr_m)) {
        std::exit(EXIT_FAILURE);
      }
    }

    PRINT_INFO("[Info]: %d IMU data loaded. \n", imu_manager->get_data_count());
  }

  /**
   * @brief Read the MoCap data and feed them to the MoCap measurement manager.
   *
   * @param imu_manager MoCap measurement manager.
   * @param imu_data_path MoCap data path.
   */
  static void ReadMoCapData(
      std::shared_ptr<mocap2gt::MoCapMeasManagerBase> mocap_manager,
      const std::string &mocap_data_path) {
    std::ifstream file(mocap_data_path);
    if (!file.is_open()) {
      PRINT_ERROR("[Error]: Failed to open mocap data file: %s \n",
                  mocap_data_path.c_str());
      std::exit(EXIT_FAILURE);
    }

    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      std::vector<double> values;

      try {
        while (std::getline(ss, value, ' ')) {
          values.push_back(std::stod(value));
        }
      } catch (const std::exception &e) {
        PRINT_ERROR(
            "[Error]: Timestamped poses in the MoCap file must be in TUM "
            "format, "
            "i.e. "
            "\"timestamp(s) tx(m) ty(m) tz(m) qx qy qz qw\" \n")
        std::exit(EXIT_FAILURE);
      }

      if (values.size() != 8) {
        PRINT_ERROR(
            "[Error]: Timestamped poses in the MoCap file must be in TUM "
            "format, "
            "i.e. "
            "\"timestamp(s) tx(m) ty(m) tz(m) qx qy qz qw\" \n")
        std::exit(EXIT_FAILURE);
      }

      double timestamp(values[0]);
      Eigen::Vector3d p_m(values[1], values[2], values[3]);
      Eigen::Quaterniond q_m(values[7], values[4], values[5], values[6]);

      mocap_manager->FeedPose(timestamp, p_m, q_m);
    }

    PRINT_INFO("[Info]: %d MoCap data loaded. \n",
               mocap_manager->get_data_count());
  }
};

}  // namespace mocap2gt
