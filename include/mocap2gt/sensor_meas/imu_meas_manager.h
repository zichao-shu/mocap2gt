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

#include <memory>
#include <vector>

#include "mocap2gt/sensor_meas/imu_preintegrator.h"
#include "mocap2gt/utils/printer.h"

namespace mocap2gt {

// Structure of IMU data.
struct ImuData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuData()
      : timestamp(-1.),
        acc_m(Eigen::Vector3d::Zero()),
        gyr_m(Eigen::Vector3d::Zero()) {}
  ImuData(const double &t, const Eigen::Vector3d &acc,
          const Eigen::Vector3d &gyr)
      : timestamp(t), acc_m(acc), gyr_m(gyr) {}

  // Timestamp in seconds.
  double timestamp;
  // Measurement of accelerometer and gyroscope.
  Eigen::Vector3d acc_m;
  Eigen::Vector3d gyr_m;

  // Comparison operator overloading, for fast binary search.
  bool operator<(const ImuData &data) const {
    return timestamp < data.timestamp;
  }
  bool operator>(const ImuData &data) const {
    return timestamp > data.timestamp;
  }
};

/**
 * @brief Class for storing and managing IMU data. Obtain the preintegration
 * result for a given time period.
 */
class ImuMeasManager {
 public:
  /**
   * @brief Construct the IMU measurement manager, initialize IMU frequency and
   * convert noise std to discrete time.
   *
   * @param imu_freq IMU frequency.
   * @param imu_noise Vector of IMU noise std in continuous time. [acc white
   * noise (m/s^2/sqrt(Hz)), acc bias walk (m/s^3/sqrt(Hz)), gyr white noise
   * (rad/s/sqrt(Hz)), gyr bias walk (rad/s^2/sqrt(Hz))]
   * @param g_magnitude Gravity magnitude.
   */
  ImuMeasManager(const double &imu_freq, const Eigen::Vector4d &imu_noise,
                 const double &g_magnitude);

  /**
   * @brief Feed IMU measurements, will append to historical vector.
   *
   * @param timestamp Timestamp of the measurement (s).
   * @param acc Acceleration part.
   * @param gyr Angular velocity part.
   * @return True if we can feed it.
   */
  bool FeedImu(const double &timestamp, const Eigen::Vector3d &acc,
               const Eigen::Vector3d &gyr);

  /**
   * @brief Get the angular velocity at a given timestamp.
   *
   * @param timestamp Target timestamp (s).
   * @param angular_vel_ptr Target angular velocity in the local frame.
   * @return True if able to get the angular velocity.
   */
  bool GetAngleVelocity(const double &timestamp,
                        Eigen::Vector3d *angular_vel_ptr);

  /**
   * @brief Retrieve IMU data for the given time period and calculate the
   * preintegration result.
   *
   * @param time_0 Start time.
   * @param time_1 End time.
   * @param lin_ba Linearization point of the accelerometer bias.
   * @param lin_bg Linearization point of gyroscope bias.
   * @param integrator_ptr Preintegration result
   * @return True if able to calculate the preintegration result.
   */
  bool Preintegrate(const double &time_0, const double &time_1,
                    const Eigen::Vector3d &lin_ba,
                    const Eigen::Vector3d &lin_bg,
                    std::shared_ptr<ImuPreintegrator> *integrator_ptr);

  // Accessors for IMU data information.
  const double &get_start_time() const { return start_time_; }
  const double &get_end_time() const { return end_time_; }
  const int get_data_count() const { return imu_data_buf_.size(); }

 private:
  /**
   * @brief Linearly interpolate between two imu data to obtain the IMU data at
   * the target timestamp.
   *
   * @param data_1 Start IMU data.
   * @param data_2 End IMU data.
   * @param target_t Target timestamp.
   * @param data_out_ptr Output IMU data.
   */
  void InterpolateImuData(const ImuData &data_0, const ImuData &data_1,
                          const double &target_t, ImuData *data_out_ptr);

  // IMU measurements historical vector.
  std::vector<ImuData> imu_data_buf_;

  // IMU noise standard deviation (discrete time).
  double acc_noise_;
  double acc_b_noise_;
  double gyr_noise_;
  double gyr_b_noise_;

  double gravity_magnitude_;

  // IMU data information.
  double start_time_ = -1.;
  double end_time_ = -1.;
};

}  // namespace mocap2gt
