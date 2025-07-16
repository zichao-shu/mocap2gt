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

namespace mocap2gt {

ImuMeasManager::ImuMeasManager(const double &imu_freq,
                               const Eigen::Vector4d &imu_noise,
                               const double &g_magnitude) {
  // Convert noise std from continuous time to discrete time.
  acc_noise_ = imu_noise[0] * sqrt(imu_freq);
  acc_b_noise_ = imu_noise[1] / sqrt(imu_freq);
  gyr_noise_ = imu_noise[2] * sqrt(imu_freq);
  gyr_b_noise_ = imu_noise[3] / sqrt(imu_freq);

  gravity_magnitude_ = g_magnitude;
}

bool ImuMeasManager::FeedImu(const double &timestamp,
                             const Eigen::Vector3d &acc,
                             const Eigen::Vector3d &gyr) {
  // Assert the timestamps are positive and in ascending order.
  if (timestamp < 0 || timestamp <= end_time_) {
    PRINT_ERROR(
        "[Error]: Feed IMU data error, timestamp is negative or does not "
        "increase: %f \n",
        timestamp);

    return false;
  }

  if (start_time_ < 0) {
    start_time_ = timestamp;
  }

  if (end_time_ > 0 && std::abs(timestamp - end_time_) >= 0.1) {
    PRINT_WARNING(
        "[Warning]: Missing IMU data for more than 0.1s after %f may lead to "
        "inaccurate trajectory estimation. \n",
        timestamp);
  }

  end_time_ = timestamp;
  imu_data_buf_.emplace_back(timestamp, acc, gyr);

  return true;
}

bool ImuMeasManager::GetAngleVelocity(const double &timestamp,
                                      Eigen::Vector3d *angular_vel_ptr) {
  // Set the default values.
  *angular_vel_ptr = Eigen::Vector3d::Zero();

  if (imu_data_buf_.size() < 2) {
    PRINT_ERROR("[Error]: Insufficient IMU measurements. \n");

    std::exit(EXIT_FAILURE);
  }

  ImuData target_data;
  target_data.timestamp = timestamp;
  auto target_ub =
      std::upper_bound(imu_data_buf_.begin(), imu_data_buf_.end(), target_data);

  if (target_ub == imu_data_buf_.begin() || target_ub == imu_data_buf_.end()) {
    PRINT_WARNING(
        "[Warning]: Specified angular velocity sampling time: %f is out of the "
        "time range of the IMU measurements: (%f, %f). \n",
        timestamp, imu_data_buf_.begin()->timestamp,
        imu_data_buf_.rbegin()->timestamp)

    return false;
  }

  if (std::abs(timestamp - (target_ub - 1)->timestamp) > 0.1 ||
      std::abs(timestamp - target_ub->timestamp) > 0.1) {
    PRINT_WARNING(
        "[Warning]: Missing IMU data for more than 0.1s around %f, unable to "
        "get an accurate angular velocity. \n",
        timestamp);

    return false;
  }

  // Start data.
  InterpolateImuData(*(target_ub - 1), *target_ub, timestamp, &target_data);
  *angular_vel_ptr = target_data.gyr_m;

  return true;
}

bool ImuMeasManager::Preintegrate(
    const double &time_0, const double &time_1, const Eigen::Vector3d &lin_ba,
    const Eigen::Vector3d &lin_bg,
    std::shared_ptr<ImuPreintegrator> *integrator_ptr) {
  // Step 1: Retrieve IMU data for the given time period.
  std::vector<ImuData> integ_data;

  if (imu_data_buf_.size() < 2) {
    PRINT_ERROR("[Error]: Insufficient IMU measurements. \n");

    std::exit(EXIT_FAILURE);
  }

  if (time_0 > time_1) {
    PRINT_ERROR(
        "[Error]: Unexpected increase in preintegration time, t_0: %f, t_1: %f "
        "\n",
        time_0, time_1);
    *integrator_ptr = nullptr;

    std::exit(EXIT_FAILURE);
  }

  // Assert the preintegration time is within the time range of the IMU.
  ImuData start_data, end_data;
  start_data.timestamp = time_0;
  end_data.timestamp = time_1;

  // Upper bound of the start IMU data.
  auto start_ub =
      std::upper_bound(imu_data_buf_.begin(), imu_data_buf_.end(), start_data);
  // Upper bound of the end IMU data.
  auto end_ub =
      std::upper_bound(imu_data_buf_.begin(), imu_data_buf_.end(), end_data);

  if (start_ub == imu_data_buf_.begin() || end_ub == imu_data_buf_.begin() ||
      start_ub == imu_data_buf_.end() || end_ub == imu_data_buf_.end()) {
    PRINT_WARNING(
        "[Warning]: Specified preintegration time: (%f, %f) is out of the time "
        "range of the IMU measurements: (%f, %f). \n",
        time_0, time_1, imu_data_buf_.begin()->timestamp,
        (imu_data_buf_.end() - 1)->timestamp)
    *integrator_ptr = nullptr;

    return false;
  }

  if (std::abs(time_0 - (start_ub - 1)->timestamp) > 0.1 ||
      std::abs(time_0 - start_ub->timestamp) > 0.1 ||
      std::abs(time_1 - (end_ub - 1)->timestamp) > 0.1 ||
      std::abs(time_1 - end_ub->timestamp) > 0.1) {
    PRINT_WARNING(
        "[Warning]: Missing IMU data for more than 0.1s around %f, unable to "
        "perform an accurate preintegration. \n",
        time_0);
    *integrator_ptr = nullptr;

    return false;
  }

  // Start data.
  InterpolateImuData(*(start_ub - 1), *start_ub, time_0, &start_data);
  integ_data.push_back(start_data);

  // Middle data.
  for (auto iter = start_ub; iter < end_ub; iter++) {
    integ_data.push_back(*iter);
  }

  // End data.
  InterpolateImuData(*(end_ub - 1), *end_ub, time_1, &end_data);
  integ_data.push_back(end_data);

  // ===========================================================================

  // Step 2: Check the IMU data.
  // Loop through and ensure we do not have an zero dt values
  for (size_t i = 0; i < integ_data.size() - 1; ++i) {
    // This shouldn not happen.
    if (integ_data.at(i + 1).timestamp < integ_data.at(i).timestamp) {
      PRINT_ERROR("[Error]: Preintegration data error, timestamp decrease. \n");

      std::exit(EXIT_FAILURE);
    }

    if (std::abs(integ_data.at(i + 1).timestamp - integ_data.at(i).timestamp) <
        1e-6) {
      PRINT_DEBUG(
          "[Debug]: Zero dt between IMU measurements %.9f and %.9f, remove the "
          "latter. \n",
          integ_data.at(i).timestamp, integ_data.at(i + 1).timestamp);
      integ_data.erase(integ_data.begin() + i);
      i--;
    }
  }

  if (integ_data.size() < 2) {
    PRINT_WARNING(
        "[Warning]: Insufficient IMU measurements within the specified "
        "preintegration time: (%f, %f). \n");
    *integrator_ptr = nullptr;

    return false;
  }

  // Preintegrate at least three data to prevent degeneration of the covariance
  // matrix.
  if (integ_data.size() == 2) {
    ImuData extra_data;
    double extra_data_time =
        (integ_data[0].timestamp + integ_data[1].timestamp) / 2;
    InterpolateImuData(integ_data[0], integ_data[1], extra_data_time,
                       &extra_data);
    integ_data.insert(integ_data.begin() + 1, extra_data);

    PRINT_DEBUG(
        "[Debug]: There are only two IMU data within (%.6f, %.6f). Generate an "
        "extra IMU data at %.6f. \n",
        integ_data.begin()->timestamp, integ_data.end()->timestamp,
        extra_data_time);
  }

  // ===========================================================================

  // Step 3: Preintegration.
  // Loop through and compute the preintegration.
  for (size_t i = 0; i < integ_data.size(); ++i) {
    double timestamp = integ_data.at(i).timestamp;
    Eigen::Vector3d acc_m = integ_data.at(i).acc_m;
    Eigen::Vector3d gyr_m = integ_data.at(i).gyr_m;

    PRINT_ALL(
        "[Debug]: Preintegration data: t(%f), acc_m(%f, %f, %f), gyr_m(%f, %f, "
        "%f). \n",
        timestamp, acc_m.x(), acc_m.y(), acc_m.z(), gyr_m.x(), gyr_m.y(),
        gyr_m.z());

    if (i == 0) {
      (*integrator_ptr)
          .reset(new ImuPreintegrator(acc_m, gyr_m, lin_ba, lin_bg, acc_noise_,
                                      acc_b_noise_, gyr_noise_, gyr_b_noise_,
                                      gravity_magnitude_));
    } else {
      double dt = timestamp - integ_data.at(i - 1).timestamp;
      (*integrator_ptr)
          ->Propagate(dt, integ_data.at(i).acc_m, integ_data.at(i).gyr_m);
    }
  }

  // ===========================================================================

  return true;
}

void ImuMeasManager::InterpolateImuData(const ImuData &data_0,
                                        const ImuData &data_1,
                                        const double &target_t,
                                        ImuData *data_out_ptr) {
  // Time-distance lambda
  double lambda =
      (target_t - data_0.timestamp) / (data_1.timestamp - data_0.timestamp);

  // Linearly interpolate between the two messages.
  data_out_ptr->timestamp = target_t;
  data_out_ptr->acc_m = (1 - lambda) * data_0.acc_m + lambda * data_1.acc_m;
  data_out_ptr->gyr_m = (1 - lambda) * data_0.gyr_m + lambda * data_1.gyr_m;
}

}  // namespace mocap2gt
