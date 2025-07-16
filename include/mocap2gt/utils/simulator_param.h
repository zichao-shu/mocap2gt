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

#include <string>

#include "mocap2gt/utils/config_parser.h"

namespace mocap2gt {

/**
 * @brief Class for saving and accessing the parameters used in B-spline
 * simulator sensor data generation.
 *
 * The frame we have defined is as follows:
 * - (M) denotes the local frame of the MoCap system, referenced to a specific
 * tracking marker.
 * - (I) denotes the local frame of the IMU.
 * - (W) denotes the world frame of the MoCap system.
 * - (G) denotes the world frame of the IMU, aligned with gravity.
 */
class SimulatorParam {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Parse the simulator parameters in constructor.
  explicit SimulatorParam(const std::string &config_path) {
    ParseConfigYaml(config_path);
  }

  // Accessors for sensor frequency.
  const double &get_mocap_freq() const { return mocap_freq_; }

  const double &get_imu_freq() const { return imu_freq_; }

  // Accessors for sensor transformation.
  const Eigen::Vector3d &get_p_MI() const { return p_MI_; }

  const Eigen::Quaterniond &get_q_MI() const { return q_MI_; }

  const Eigen::Quaterniond &get_q_WG() const { return q_WG_; }

  const double &get_toff_MI() const { return toff_MI_; }

  const double &get_gravity_magnitude() const { return gravity_magnitude_; }

  // Accessors for sensor noise.
  const Eigen::Vector3d &get_mocap_trans_noise() const {
    return mocap_trans_noise_;
  }

  const Eigen::Vector3d &get_mocap_rot_noise() const {
    return mocap_rot_noise_;
  }

  const Eigen::Vector4d &get_imu_noise() const { return imu_noise_; }

  const double &get_toff_drift_noise() const { return toff_drift_noise_; }

 private:
  // Parse the .yaml simulator parameters file.
  void ParseConfigYaml(const std::string &config_path);

  // yaml-cpp node.
  YAML::Node config_;

  // Frequency of generated data.
  double mocap_freq_ = 300.;
  double imu_freq_ = 500.;

  // Transformation between IMU and MoCap in generated data.
  // Transform from I(IMU local frame) to M(MoCap local frame).
  Eigen::Vector3d p_MI_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_MI_ = Eigen::Quaterniond::Identity();
  // Transform from G(IMU world frame, gravity aligned) to W(MoCap world frame).
  Eigen::Quaterniond q_WG_ = Eigen::Quaterniond::Identity();
  // Time offset between IMU and MoCap (IMU time = mocap time + toff_MI).
  double toff_MI_ = 0.;
  double gravity_magnitude_ = 9.8;

  // Noise in generated data.
  // Discrete time MoCap noise std.
  Eigen::Vector3d mocap_trans_noise_ = Eigen::Vector3d(0., 0., 0.);
  Eigen::Vector3d mocap_rot_noise_ = Eigen::Vector3d(0., 0., 0.);
  // Continuous time IMU noise std. [acc white noise, acc bias walk, gyr white
  // noise, gyr bias walk]
  Eigen::Vector4d imu_noise_ = Eigen::Vector4d(0., 0., 0., 0.);
  // Continuous time drift noise std (ms/s/sqrt(Hz)), nota that we assume the
  // time drift to be a random walk.
  double toff_drift_noise_ = 0.0;
};

}  // namespace mocap2gt
