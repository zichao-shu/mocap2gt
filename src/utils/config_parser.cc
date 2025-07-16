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

namespace mocap2gt {

void ConfigParser::ParseConfigYaml(const std::string &config_path) {
  try {
    config_ = YAML::LoadFile(config_path);
  } catch (const std::exception &e) {
    PRINT_ERROR("[Error]: Failed to load the config file: %s \n",
                config_path.c_str());
    std::exit(EXIT_FAILURE);
  }

  // ===========================================================================

  // Step 1: Parse the optimization options.
  PRINT_INFO("Parse the optimization options: \n");
  if (ParseValue(config_, "est_traj_freq", &est_traj_freq_)) {
    PRINT_INFO("[Info]: Est trajectory frequency set to: %.1f\n",
               est_traj_freq_);
  } else {
    PRINT_INFO(
        "[Info]: Est trajectory frequency not specified, set to default: "
        "%.1f\n",
        est_traj_freq_);
  }

  if (ParseValue(config_, "relin_iter_count", &relin_iter_count_)) {
    PRINT_INFO("[Info]: Relinearization iter count set to: %d\n",
               relin_iter_count_);
  } else {
    PRINT_INFO(
        "[Info]: Relinearization iter count not specified, set to default: "
        "%d\n",
        relin_iter_count_);
  }

  if (ParseValue(config_, "use_spline_model_flag", &use_spline_model_flag_)) {
    PRINT_INFO("[Info]: Flag for using cubic B-spline MoCap model set to: %d\n",
               use_spline_model_flag_);
  } else {
    PRINT_INFO(
        "[Info]: Flag for using cubic B-spline MoCap model not specified, set "
        "to default: %d\n",
        use_spline_model_flag_);
  }

  if (ParseValue(config_, "est_T_MI_flag", &est_T_MI_flag_)) {
    PRINT_INFO("[Info]: Flag for estimating T_MI set to: %d\n", est_T_MI_flag_);
  } else {
    PRINT_INFO(
        "[Info]: Flag for estimating T_MI not specified, set to default: %d\n",
        est_T_MI_flag_);
  }

  if (ParseValue(config_, "est_q_WG_flag", &est_q_WG_flag_)) {
    PRINT_INFO("[Info]: Flag for estimating q_WG set to: %d\n", est_q_WG_flag_);
  } else {
    PRINT_INFO(
        "[Info]: Flag for estimating q_WG not specified, set to default: %d\n",
        est_q_WG_flag_);
  }

  if (ParseValue(config_, "est_toff_MI_flag", &est_toff_MI_flag_)) {
    PRINT_INFO("[Info]: Flag for estimating toff_MI set to: %d\n",
               est_toff_MI_flag_);
  } else {
    PRINT_INFO(
        "[Info]: Flag for estimating toff_MI not specified, set to default: "
        "%d\n",
        est_toff_MI_flag_);
  }

  if (ParseValue(config_, "variable_toff_flag", &variable_toff_flag_)) {
    PRINT_INFO("[Info]: Flag for using variable time offset set to: %d\n",
               variable_toff_flag_);
  } else {
    PRINT_INFO(
        "[Info]: Flag for using variable time offset not specified, set to "
        "default: %d\n",
        variable_toff_flag_);
  }

  if (variable_toff_flag_ == true) {
    if (ParseValue(config_, "toff_control_point_interval",
                   &toff_control_point_interval_)) {
      PRINT_INFO(
          "[Info]: Interval of time offset control points set to: %.2f\n",
          toff_control_point_interval_);
    } else {
      PRINT_INFO(
          "[Info]: Interval of time offset control points not specified, set "
          "to default: %.2f\n",
          toff_control_point_interval_);
    }
  }

  if (ParseValue(config_, "max_toff", &max_toff_)) {
    if (max_toff_ < 0.1) {
      PRINT_INFO("[Info]: Max change in time offset set is too small. \n");
      max_toff_ = 0.1;
    }
    if (max_toff_ > 0.5) {
      PRINT_INFO("[Info]: Max change in time offset set is too big. \n");
      max_toff_ = 0.5;
    }
    PRINT_INFO("[Info]: Max change in time offset set to: %.2f\n", max_toff_);
  } else {
    PRINT_INFO(
        "[Info]: Max change in time offset not specified, set to default: "
        "%.2f\n",
        max_toff_);
  }

  if (ParseValue(config_, "vel_init_range", &vel_init_range_)) {
    if (vel_init_range_ < 0.01) {
      PRINT_INFO(
          "[Info]: Time range for initializing the IMU velocity is too small. "
          "\n");
      vel_init_range_ = 0.01;
    }
    if (vel_init_range_ >= 2 * max_toff_) {
      PRINT_INFO(
          "[Info]: Time range for initializing the IMU velocity must be "
          "smaller than twice the max change in time offset. \n");
      vel_init_range_ = max_toff_;
    }
    PRINT_INFO(
        "[Info]: Time range for initializing the IMU velocity set to: %.2f\n",
        vel_init_range_);
  } else {
    PRINT_INFO(
        "[Info]: Time range for initializing the IMU velocity not specified, "
        "set to default: %.2f\n",
        vel_init_range_);
  }

  // ===========================================================================

  // Step 2: Parse the initial guesses.
  PRINT_INFO("Parse the initial guesses: \n");

  if (ParseValue(config_, "use_initializer_flag", &use_initializer_flag_)) {
    if (use_initializer_flag_) {
      PRINT_INFO(
          "[Info]: Flag for use initializer set to: %d, unfixed calibration "
          "parameters will be initialized. \n",
          use_initializer_flag_);
    } else {
      PRINT_INFO(
          "[Info]: Flag for use initializer set to: %d, we will use "
          "user-specified initial guesses. \n",
          use_initializer_flag_);
    }
  } else {
    PRINT_INFO(
        "[Info]: Flag for use initializer not specified, set to default: %d, "
        "unfixed calibration parameters will be initialized. \n",
        use_initializer_flag_);
  }

  if (!use_initializer_flag_ || !est_T_MI_flag_) {
    if (ParseValue(config_, "p_MI_init", &p_MI_init_)) {
      PRINT_INFO(
          "[Info]: Initial guess of extrinsic p_MI set to: [%.6f, %.6f, "
          "%.6f]\n",
          p_MI_init_.x(), p_MI_init_.y(), p_MI_init_.z());
    } else {
      PRINT_INFO(
          "[Info]: Initial guess of extrinsic p_MI not specified, set to "
          "default: [%.6f, %.6f, %.6f]\n",
          p_MI_init_.x(), p_MI_init_.y(), p_MI_init_.z());
    }

    if (ParseValue(config_, "q_MI_init", &q_MI_init_)) {
      q_MI_init_.normalize();
      PRINT_INFO(
          "[Info]: Initial guess of extrinsic q_MI set to: [%.6f, %.6f, %.6f, "
          "%.6f]\n",
          q_MI_init_.x(), q_MI_init_.y(), q_MI_init_.z(), q_MI_init_.w());
    } else {
      PRINT_INFO(
          "[Info]: Initial guess of extrinsic q_MI not specified, set to "
          "default: [%.6f, %.6f, %.6f, %.6f]\n",
          q_MI_init_.x(), q_MI_init_.y(), q_MI_init_.z(), q_MI_init_.w());
    }
  }

  if (!use_initializer_flag_ || !est_q_WG_flag_) {
    if (ParseValue(config_, "q_WG_init", &q_WG_init_)) {
      q_WG_init_.normalize();
      PRINT_INFO(
          "[Info]: Initial guess of gravity alignment q_WG set to: [%.6f, "
          "%.6f, "
          "%.6f, %.6f]\n",
          q_WG_init_.x(), q_WG_init_.y(), q_WG_init_.z(), q_WG_init_.w());
    } else {
      PRINT_INFO(
          "[Info]: Initial guess of gravity alignment q_WG not specified, set "
          "to "
          "default: [%.6f, %.6f, %.6f, %.6f]\n",
          q_WG_init_.x(), q_WG_init_.y(), q_WG_init_.z(), q_WG_init_.w());
    }
  }

  if (!use_initializer_flag_ || !est_toff_MI_flag_) {
    if (ParseValue(config_, "toff_MI_init", &toff_MI_init_)) {
      PRINT_INFO("[Info]: Initial guess of time offset toff_MI set to: %.6f\n",
                 toff_MI_init_);
    } else {
      PRINT_INFO(
          "[Info]: Initial guess of time offset toff_MI not specified, set to "
          "default: %.6f\n",
          toff_MI_init_);
    }
  }

  if (ParseValue(config_, "gravity_magnitude", &gravity_magnitude_)) {
    PRINT_INFO("[Info]: Gravity magnitude set to: %.6f\n", gravity_magnitude_);
  } else {
    PRINT_INFO(
        "[Info]: Gravity magnitude not specified, set to default: %.6f\n",
        gravity_magnitude_);
  }

  // ===========================================================================

  // Step 3: Parse the sensor settings.
  PRINT_INFO("Parse the sensor settings: \n");
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

  if (ParseValue(config_, "imu_freq", &imu_freq_)) {
    PRINT_INFO("[Info]: IMU frequency set to: %.1f\n", imu_freq_);
  } else {
    PRINT_INFO("[Info]: IMU frequency not specified, set to default: %.1f\n",
               imu_freq_);
  }

  // ===========================================================================
}

}  // namespace mocap2gt
