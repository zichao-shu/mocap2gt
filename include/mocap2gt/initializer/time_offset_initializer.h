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

#include "mocap2gt/sensor_meas/imu_meas_manager.h"
#include "mocap2gt/sensor_meas/mocap_meas_manager_linear.h"
#include "mocap2gt/sensor_meas/mocap_meas_manager_spline.h"

namespace mocap2gt {

/**
 * @brief Classes for estimating the initial guess of time offset (toff_MI),
 * which we implement based on the constraint of screw congruence theorem.
 * Specifically, we compute the angular velocity signal and use correlation
 * analysis to obtain the time offset result.
 */
class TimeOffsetInitializer {
 public:
  /**
   * @brief Construct the time offset initializer, pass the pointers and
   * initialize the start and end times.
   *
   * @param i_manager IMU measurement manager.
   * @param m_manager MoCap measurement manager.
   * @param truncate_time Truncate time of the data.
   */
  TimeOffsetInitializer(const std::shared_ptr<ImuMeasManager>& i_manager,
                        const std::shared_ptr<MoCapMeasManagerBase>& m_manager,
                        const double& truncate_time);

  /**
   * @brief Interface function to estimate the initial guess of the time offset
   * based on contraint of screw congruence theorem.
   *
   * @param toff_MI_ptr Time offset initial guess, t_IMU = t_MoCap + toff_MI.
   * @param sample_interval Time interval for calculating angular velocity.
   * @param smooth_kernal_size The size of the angular velocity smoothing
   * kernel.
   * @return True if able to initialize the time offset.
   */
  bool EstimateToff(double* toff_MI_ptr, const double& sample_interval = 0.01,
                    const int& smooth_kernal_size = 5);

 private:
  /**
   * @brief Calculate the correlation of two discrete signal.
   *
   * @param signal_0 Input signal 0.
   * @param signal_1 Input signal 1, which we use as the kernel.
   * @param output Output correlation result.
   * @param same_size Whether to keep the same size as input signal 0.
   */
  void SignalCorrelation(const Eigen::MatrixXd& signal_0,
                         const Eigen::RowVectorXd& signal_1,
                         Eigen::MatrixXd* output, const bool& same_size);

  /**
   * @brief Get the index of the maximum of the correlation function. To improve
   * the accuracy, we use a quadratic polynomial fitting for approximation.
   *
   * @param correlation_function Input correlation function.
   * @param max_index_ptr Maximum index result.
   * @param fitting_half_range The half range for polynomial fitting
   */
  void CorrelationMaxIndex(const Eigen::RowVectorXd& correlation_function,
                           double* max_index_ptr,
                           const int& fitting_half_range = 3);

  // IMU manager used to obtain IMU measurements.
  std::shared_ptr<ImuMeasManager> imu_manager_;
  // MoCap manager used to obtain MoCap measurements.
  std::shared_ptr<MoCapMeasManagerBase> mocap_manager_;

  // Start/end times of measurements.
  double imu_start_time_;
  double imu_end_time_;
  double mocap_start_time_;
  double mocap_end_time_;
};

}  // namespace mocap2gt
