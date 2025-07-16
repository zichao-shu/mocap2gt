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

#include "mocap2gt/utils/simulator_param.h"

namespace mocap2gt {

void SimulatorParam::ParseConfigYaml(const std::string &config_path) {
  try {
    config_ = YAML::LoadFile(config_path);
  } catch (const std::exception &e) {
    PRINT_ERROR("[Error]: Failed to load the config_ file: %s \n",
                config_path.c_str());
    std::exit(EXIT_FAILURE);
  }

  // ===========================================================================

  // Step 1: Parse the sensor frequency.
  if (ParseValue(config_, "mocap_freq", &mocap_freq_)) {
    PRINT_INFO("[Info]: MoCap frequency set to: %.1f \n", mocap_freq_);
  } else {
    PRINT_INFO("[Info]: MoCap frequency not specified, set to default: %.1f\n",
               mocap_freq_);
  }

  if (ParseValue(config_, "imu_freq", &imu_freq_)) {
    PRINT_INFO("[Info]: IMU frequency set to: %.1f \n", imu_freq_);
  } else {
    PRINT_INFO("[Info]: IMU frequency not specified, set to default: %.1f\n",
               imu_freq_);
  }

  // ===========================================================================

  // Step 2: Parse the sensor transformation.
  if (ParseValue(config_, "p_MI", &p_MI_)) {
    PRINT_INFO("[Info]: Extrinsic p_MI set to: [%.6f, %.6f, %.6f]\n", p_MI_.x(),
               p_MI_.y(), p_MI_.z());
  } else {
    PRINT_INFO(
        "[Info]: Extrinsic p_MI not specified, set to default: [%.6f, %.6f, "
        "%.6f]\n",
        p_MI_.x(), p_MI_.y(), p_MI_.z());
  }

  if (ParseValue(config_, "q_MI", &q_MI_)) {
    q_MI_.normalize();
    PRINT_INFO("[Info]: Extrinsic q_MI set to: [%.6f, %.6f, %.6f, %.6f]\n",
               q_MI_.x(), q_MI_.y(), q_MI_.z(), q_MI_.w());
  } else {
    PRINT_INFO(
        "[Info]: Extrinsic q_MI not specified, set to default: [%.6f, %.6f, "
        "%.6f, %.6f]\n",
        q_MI_.x(), q_MI_.y(), q_MI_.z(), q_MI_.w());
  }

  if (ParseValue(config_, "q_WG", &q_WG_)) {
    q_WG_.normalize();
    PRINT_INFO(
        "[Info]: Gravity alignment q_WG set to: [%.6f, %.6f, %.6f, %.6f]\n",
        q_WG_.x(), q_WG_.y(), q_WG_.z(), q_WG_.w());
  } else {
    PRINT_INFO(
        "[Info]: Gravity alignment q_WG not specified, set to default: [%.6f, "
        "%.6f, %.6f, %.6f]\n",
        q_WG_.x(), q_WG_.y(), q_WG_.z(), q_WG_.w());
  }

  if (ParseValue(config_, "toff_MI", &toff_MI_)) {
    PRINT_INFO("[Info]: Time offset toff_MI set to: %.6f\n", toff_MI_);
  } else {
    PRINT_INFO(
        "[Info]: Time offset toff_MI not specified, set to default: %.6f\n",
        toff_MI_);
  }

  if (ParseValue(config_, "gravity_magnitude", &gravity_magnitude_)) {
    PRINT_INFO("[Info]: Gravity magnitude set to: %.6f\n", gravity_magnitude_);
  } else {
    PRINT_INFO(
        "[Info]: Gravity magnitude not specified, set to default: %.6f\n",
        gravity_magnitude_);
  }

  // ===========================================================================

  // Step3: Parse the sensor noise.
  if (ParseValue(config_, "mocap_trans_noise", &mocap_trans_noise_)) {
    PRINT_INFO("[Info]: Mocap translation noise set to: [%.6f, %.6f, %.6f]\n",
               mocap_trans_noise_.x(), mocap_trans_noise_.y(),
               mocap_trans_noise_.z());
  } else {
    PRINT_INFO(
        "[Info]: Mocap translation noise not specified, set to default: [%.6f, "
        "%.6f, %.6f]\n",
        mocap_trans_noise_.x(), mocap_trans_noise_.y(), mocap_trans_noise_.z());
  }

  if (ParseValue(config_, "mocap_rot_noise", &mocap_rot_noise_)) {
    PRINT_INFO("[Info]: Mocap rotation noise set to: [%.6f, %.6f, %.6f]\n",
               mocap_rot_noise_.x(), mocap_rot_noise_.y(),
               mocap_rot_noise_.z());
  } else {
    PRINT_INFO(
        "[Info]: Mocap rotation noise not specified, set to default: [%.6f, "
        "%.6f, %.6f]\n",
        mocap_rot_noise_.x(), mocap_rot_noise_.y(), mocap_rot_noise_.z());
  }

  if (ParseValue(config_, "imu_noise", &imu_noise_)) {
    PRINT_INFO("[Info]: IMU noise set to: [%.6f, %.6f, %.6f, %.6f]\n",
               imu_noise_[0], imu_noise_[1], imu_noise_[2], imu_noise_[3]);
  } else {
    PRINT_INFO(
        "[Info]: IMU noise not specified, set to default: [%.6f, %.6f, %.6f, "
        "%.6f]\n",
        imu_noise_[0], imu_noise_[1], imu_noise_[2], imu_noise_[3]);
  }

  if (ParseValue(config_, "toff_drift_noise", &toff_drift_noise_)) {
    PRINT_INFO("[Info]: Time offset drift noise set to: %.6f\n",
               toff_drift_noise_);
  } else {
    PRINT_INFO(
        "[Info]: Time offset drift noise not specified, set to default: %.6f\n",
        toff_drift_noise_);
  }

  // ===========================================================================
}

}  // namespace mocap2gt
