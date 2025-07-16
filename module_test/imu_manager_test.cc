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

#include "mocap2gt/sensor_meas/imu_meas_manager.h"

int main() {
  mocap2gt::Printer::SetPrintLevel(mocap2gt::Printer::PrintLevel::ALL);

  constexpr double kImuFrequency = 10.;
  constexpr double kGravityMagnitude = 9.81;
  constexpr int kTestDataCount = 6;
  constexpr int kDataStartTime = 100.;

  // Initialize the IMU measurement manager->
  std::shared_ptr<mocap2gt::ImuMeasManager> manager(
      new mocap2gt::ImuMeasManager(kImuFrequency, Eigen::Vector4d::Random(),
                                   kGravityMagnitude));

  // Feed some test data to the IMU manager with a time range of (100, 100.5).
  PRINT_DEBUG("================== Feed IMU for test. ==================\n");
  for (int i = 0; i < kTestDataCount; ++i) {
    double timestamp = kDataStartTime + i / kImuFrequency;
    Eigen::Vector3d acc_m = Eigen::Vector3d::Ones() * i;
    Eigen::Vector3d gyr_m = Eigen::Vector3d::Ones() * i * 2;

    manager->FeedImu(timestamp, acc_m, gyr_m);
    PRINT_DEBUG(
        "[Debug]: Data %d: t(%f), acc_m(%f, %f, %f), gry_m(%f, %f, %f). \n", i,
        timestamp, acc_m.x(), acc_m.y(), acc_m.z(), gyr_m.x(), gyr_m.y(),
        gyr_m.z());
  }
  PRINT_DEBUG(
      "[Debug]: Feed a total of %d data, with timestamps ranging from %f to "
      "%f. \n",
      manager->get_data_count(), manager->get_start_time(),
      manager->get_end_time());

  // ===========================================================================

  PRINT_DEBUG("================== Test IMU manager. ==================\n");
  PRINT_DEBUG(
      "[Debug]: Below, we will use the manager to perform preintegration and "
      "test whether the IMU data has been retrieved correctly"
      "by examining the output information. \n")
  std::shared_ptr<mocap2gt::ImuPreintegrator> integrator;
  // Preintegration.
  double preinteg_start_time = kDataStartTime + 0.09;
  double preinteg_end_time = kDataStartTime + 0.21;
  PRINT_DEBUG("[Debug]: Preintegrate from %f to %f : \n", preinteg_start_time,
             preinteg_end_time);
  manager->Preintegrate(preinteg_start_time, preinteg_end_time,
                        Eigen::Vector3d::Random(), Eigen::Vector3d::Random(),
                        &integrator);

  // Test if the manager can detect preintegration time exceeding time range.
  preinteg_start_time = kDataStartTime - 0.01;
  preinteg_end_time = kDataStartTime + 0.1;
  PRINT_DEBUG("[Debug]: Preintegrate from %f to %f : \n", preinteg_start_time,
             preinteg_end_time);
  manager->Preintegrate(preinteg_start_time, preinteg_end_time,
                        Eigen::Vector3d::Random(), Eigen::Vector3d::Random(),
                        &integrator);

  // Test if the manager can detect preintegration time exceeding time range.
  preinteg_start_time = kDataStartTime + 0.1;
  preinteg_end_time = kDataStartTime + 0.6;
  PRINT_DEBUG("[Debug]: Preintegrate from %f to %f : \n", preinteg_start_time,
             preinteg_end_time);
  manager->Preintegrate(preinteg_start_time, preinteg_end_time,
                        Eigen::Vector3d::Random(), Eigen::Vector3d::Random(),
                        &integrator);

  // Test if the manager can eliminate redundant preintegration data, and
  // prevent degeneration.
  preinteg_start_time = kDataStartTime + 0.1 - 1e-9;
  preinteg_end_time = kDataStartTime + 0.2 + 1e-9;
  PRINT_DEBUG("[Debug]: Preintegrate from %.9f to %.9f : \n", preinteg_start_time,
             preinteg_end_time);
  manager->Preintegrate(preinteg_start_time, preinteg_end_time,
                        Eigen::Vector3d::Random(), Eigen::Vector3d::Random(),
                        &integrator);

  // ===========================================================================
}
