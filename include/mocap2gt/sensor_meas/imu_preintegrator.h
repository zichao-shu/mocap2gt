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

#include <Eigen/Eigen>
#include <vector>

#include "mocap2gt/utils/geometry_utils.h"

namespace mocap2gt {

/**
 * @brief Class for propagating the preintegration measurement, covariance, and
 * linearization result using the midpoint integration model.
 */
class ImuPreintegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct the IMU preintegrator, initialize the information of the
   * integrator.
   *
   * @param acc_start Start value of acceleration.
   * @param gyr_start Start value of gyroscope.
   * @param ba Linearization point of the accelerometer bias.
   * @param bg Linearization point of gyroscope bias.
   * @param acc_noise Accelerometer measurement noise standard deviation (m/s^2,
   * discrete time).
   * @param acc_b_noise Accelerometer bias random work noise standard deviation
   * (m/s^2, discrete time).
   * @param gyr_noise Gyroscope measurement noise standard deviation (rad/s,
   * discrete time).
   * @param gyr_b_noise Gyroscope bias random work noise standard deviation
   * (rad/s, discrete time).
   * @param g_magnitude Gravity magnitude.
   */
  ImuPreintegrator(const Eigen::Vector3d &acc_start,
                   const Eigen::Vector3d &gyr_start, const Eigen::Vector3d &ba,
                   const Eigen::Vector3d &bg, const double &acc_noise,
                   const double &acc_b_noise, const double &gyr_noise,
                   const double &gyr_b_noise, const double &g_magnitude);

  /**
   * @brief Single-step propagation and call the midpoint integration function.
   *
   * @param dt Time interval from last measurement.
   * @param acc Current accelerometer measurement.
   * @param gyr Current gyroscope measurement.
   */
  void Propagate(const double &dt, const Eigen::Vector3d &acc,
                 const Eigen::Vector3d &gyr);

  // Accessors for preintegration information.
  const Eigen::Vector3d &get_alpha_curr() { return alpha_curr_; }

  const Eigen::Quaterniond &get_q_curr() { return q_curr_; }

  const Eigen::Vector3d &get_beta_curr() { return beta_curr_; }

  const Eigen::Matrix<double, 15, 15> &get_meas_covariance() {
    return meas_covariance_;
  }

  const Eigen::Matrix<double, 15, 15> &get_state_jacobian() {
    return state_jacobian_;
  }

  const Eigen::Vector3d &get_gravity_in_G() { return gravity_in_G_; }

  const Eigen::Vector3d &get_linearized_ba() { return linearized_ba_; }

  const Eigen::Vector3d &get_linearized_bg() { return linearized_bg_; }

  const double &get_dt_sum() { return dt_sum_; }

  // For test only, check the Jacobian and preintegration measurement.
  friend void CheckJacobian(ImuPreintegrator *integrator_ptr, const double &dt,
                            const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &gyr);

 private:
  // Midpoint integration, updating preintegration measurement, covariance and
  // Jacobian.
  void MidPointIntegrate();

  // Time intervals historical vector.
  std::vector<double> dt_buf_;
  // Accelerometer measurements historical vector.
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      acc_data_buf_;
  // Gyroscope measurements historical vector.
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      gyr_data_buf_;

  // The measurement of accelerometer and gyroscope before and after the current
  // propagation interval.
  Eigen::Vector3d acc_m_0_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_m_0_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_m_1_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_m_1_ = Eigen::Vector3d::Zero();
  // Backup of the initial values of the accelerometer and gyroscope for
  // re-propagation.
  Eigen::Vector3d acc_m_start_;
  Eigen::Vector3d gyr_m_start_;

  // Preintegration measurement after the last propagation.
  Eigen::Vector3d alpha_last_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_last_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d beta_last_ = Eigen::Vector3d::Zero();
  // Preintegration measurement after the current propagation.
  Eigen::Vector3d alpha_curr_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_curr_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d beta_curr_ = Eigen::Vector3d::Zero();

  // Covariance of the preintegration measurement, [alpha, q, beta, ba,
  // bg]_15*15.
  Eigen::Matrix<double, 15, 15> meas_covariance_ =
      Eigen::Matrix<double, 15, 15>::Zero();
  // Covariance of the input noise, [acc_0_n, gyr_0_n, acc_1_n, gyr_1_n,
  // acc_0_b_n, gyr_0_b_n]_18*18.
  Eigen::Matrix<double, 18, 18> noise_covariance_;
  // Jacobian of state propagation, [alpha, q, beta, ba, bg]_15*15.
  Eigen::Matrix<double, 15, 15> state_jacobian_ =
      Eigen::Matrix<double, 15, 15>::Identity();
  // Jacobian of single-step state propagation, [alpha, q, beta, ba, bg]_15*15.
  Eigen::Matrix<double, 15, 15> step_state_jacobian_ =
      Eigen::Matrix<double, 15, 15>::Identity();
  // Jacobian of single-step error propagation, [alpha, q, beta, ba,
  // bg]_15*[acc_0_n, gyr_0_n, acc_1_n, gyr_1_n, acc_0_b_n, gyr_0_b_n]_18.
  Eigen::Matrix<double, 15, 18> step_noise_jacobian_ =
      Eigen::Matrix<double, 15, 18>::Identity();

  // Gravity vector in world frame G(gravity aligned).
  Eigen::Vector3d gravity_in_G_;

  // Linearization point of the accelerometer bias.
  Eigen::Vector3d linearized_ba_;
  // Linearization point of gyroscope bias.
  Eigen::Vector3d linearized_bg_;

  // The time interval of the current propagation.
  double dt_curr_ = 0.;
  // The sum of time intervals.
  double dt_sum_ = 0.;
};

// Re-declaring the friend function inside the namespace, for test only.
void CheckJacobian(ImuPreintegrator *integrator_ptr, const double &dt,
                   const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);

}  // namespace mocap2gt
