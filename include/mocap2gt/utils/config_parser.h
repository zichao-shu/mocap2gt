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

#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>
#include <string>

#include "mocap2gt/utils/printer.h"

namespace mocap2gt {

// Template function for parsing individual value.
template <typename T>
inline bool ParseValue(YAML::Node config, const std::string &key,
                       T *value_ptr) {
  if (config[key]) {
    *value_ptr = config[key].as<T>();
    return true;
  } else {
    return false;
  }
}

// Template specialization.
// Parse the Eigen::Vector3d.
template <>
inline bool ParseValue<Eigen::Vector3d>(YAML::Node config,
                                        const std::string &key,
                                        Eigen::Vector3d *value_ptr) {
  if (config[key] && config[key].IsSequence() && config[key].size() == 3) {
    for (int i = 0; i < 3; ++i) {
      (*value_ptr)[i] = config[key][i].as<double>();
    }
    return true;
  } else {
    return false;
  }
}

// Parse the Eigen::Vector4d.
template <>
inline bool ParseValue<Eigen::Vector4d>(YAML::Node config,
                                        const std::string &key,
                                        Eigen::Vector4d *value_ptr) {
  if (config[key] && config[key].IsSequence() && config[key].size() == 4) {
    for (int i = 0; i < 4; ++i) {
      (*value_ptr)[i] = config[key][i].as<double>();
    }
    return true;
  } else {
    return false;
  }
}

// Parse the Eigen::Quaterniond.
template <>
inline bool ParseValue<Eigen::Quaterniond>(YAML::Node config,
                                           const std::string &key,
                                           Eigen::Quaterniond *value_ptr) {
  if (config[key] && config[key].IsSequence() && config[key].size() == 4) {
    for (int i = 0; i < 4; ++i) {
      value_ptr->coeffs()[i] = config[key][i].as<double>();
    }
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Utility class for parsing the .yaml config file, saving the parameters
 * and accessing the parameters.
 *
 * The frame we have defined is as follows:
 * - (M) denotes the local frame of the MoCap system, referenced to a specific
 * tracking marker.
 * - (I) denotes the local frame of the IMU.
 * - (W) denotes the world frame of the MoCap system.
 * - (G) denotes the world frame of the IMU, aligned with gravity.
 */
class ConfigParser {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Parse the config in constructor.
  explicit ConfigParser(const std::string &config_path) {
    ParseConfigYaml(config_path);
  }

  // Accessors for optimization config.
  const double &get_est_traj_freq() const { return est_traj_freq_; }

  const int &get_relin_iter_count() const { return relin_iter_count_; }

  const bool &get_use_spline_model_flag() const {
    return use_spline_model_flag_;
  }

  const bool &get_est_T_MI_flag() const { return est_T_MI_flag_; }

  const bool &get_est_q_WG_flag() const { return est_q_WG_flag_; }

  const bool &get_est_toff_MI_flag() const { return est_toff_MI_flag_; }

  const bool &get_variable_toff_flag() const { return variable_toff_flag_; }

  const double &get_toff_control_point_interval() const {
    return toff_control_point_interval_;
  }

  const double &get_max_toff() const { return max_toff_; }

  const double &get_vel_init_range() const { return vel_init_range_; }

  // Accessors for initial guess.
  const bool &get_use_initializer_flag() const { return use_initializer_flag_; }

  const Eigen::Vector3d &get_p_MI_init() const { return p_MI_init_; }

  const Eigen::Quaterniond &get_q_MI_init() const { return q_MI_init_; }

  const Eigen::Quaterniond &get_q_WG_init() const { return q_WG_init_; }

  const double &get_toff_MI_init() const { return toff_MI_init_; }

  const double &get_gravity_magnitude() const { return gravity_magnitude_; }

  // Accessors for sensor setting.
  const Eigen::Vector3d &get_mocap_trans_noise() { return mocap_trans_noise_; }

  const Eigen::Vector3d &get_mocap_rot_noise() const {
    return mocap_rot_noise_;
  }

  const Eigen::Vector4d &get_imu_noise() const { return imu_noise_; }

  const double &get_imu_freq() const { return imu_freq_; }

 private:
  // Parse the .yaml config file.
  void ParseConfigYaml(const std::string &config_path);

  // yaml-cpp node.
  YAML::Node config_;

  // Optimization options.
  double est_traj_freq_ = 100.;
  int relin_iter_count_ = 1;
  bool use_spline_model_flag_ = true;
  bool est_T_MI_flag_ = true;
  bool est_q_WG_flag_ = true;
  bool est_toff_MI_flag_ = true;
  bool variable_toff_flag_ = false;
  double toff_control_point_interval_ = 60.;
  double max_toff_ = 0.25;
  double vel_init_range_ = 0.05;

  // Initial guesses.
  bool use_initializer_flag_ = true;
  // Transform from I(IMU local frame) to M(MoCap local frame).
  Eigen::Vector3d p_MI_init_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_MI_init_ = Eigen::Quaterniond::Identity();
  // Transform from G(IMU world frame, gravity aligned) to W(MoCap world frame).
  Eigen::Quaterniond q_WG_init_ = Eigen::Quaterniond::Identity();
  // Time offset between IMU and MoCap (IMU time = mocap time + toff_MI).
  double toff_MI_init_ = 0.;
  // Gravity magnitude, will not be optimized.
  double gravity_magnitude_ = 9.8;

  // Sensor settings.
  // Discrete time MoCap noise std.
  Eigen::Vector3d mocap_trans_noise_ = Eigen::Vector3d(5e-4, 5e-4, 5e-4);
  Eigen::Vector3d mocap_rot_noise_ = Eigen::Vector3d(5e-2, 5e-2, 5e-2);
  // Continuous time IMU noise std. [acc_noise, acc_bias_random_walk, gyr_noise,
  // gyr_bias_random_wak]
  Eigen::Vector4d imu_noise_ = Eigen::Vector4d(2e-2, 2e-3, 2e-4, 2e-5);
  double imu_freq_ = 100.;
};

}  // namespace mocap2gt
