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

#include "mocap2gt/sensor_meas/mocap_meas_manager_base.h"

namespace mocap2gt {

/**
 * @brief Derived class for MoCap data management, utilizing linear pose
 * interpolation. One can obtain the MoCap measurement, Jacobian and the
 * covariance at the given time.
 */
class MoCapMeasManagerLinear : public MoCapMeasManagerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct the MoCap measurement manager, initialize the MoCap noise
   * covariance matrix.
   *
   * @param trans_noise_v Standard deviation vector of MoCap translation noise.
   * @param rot_noise_v Standard deviation vector of MoCap rotation noise.
   */
  MoCapMeasManagerLinear(const Eigen::Vector3d &trans_noise_v,
                         const Eigen::Vector3d &rot_noise_v)
      : MoCapMeasManagerBase(trans_noise_v, rot_noise_v) {}

  /**
   * @brief Interpolate to obtain the pose measurement at the given time.
   *
   * @param timestamp Target timestamp (s).
   * @param p_ptr Translation part (p_WM) of the target pose.
   * @param q_ptr Rotation part (q_WM) of the target pose.
   * @return True if able to get the pose.
   */
  bool GetPose(const double &timestamp, Eigen::Vector3d *p_ptr,
               Eigen::Quaterniond *q_ptr) override;

  /**
   * @brief Get the angular velocity at the given timestamp.
   *
   * @param timestamp Target timestamp (s).
   * @param angular_vel_ptr Target angular velocity in the local frame.
   * @return True if able to get the angular velocity.
   */
  bool GetAngleVelocity(const double &timestamp,
                        Eigen::Vector3d *angular_vel_ptr) override;

  /**
   * @brief Interpolate to obtain the pose measurement, covariance and the
   * Jacobian at the given time.
   *
   * @param timestamp Target timestamp (s).
   * @param p_ptr Translation part (p_WM) of the target pose.
   * @param q_ptr Rotation part (q_WM) of the target pose.
   * @param cov_ptr Covariance of the target pose.
   * @param J_pose_toff_ptr The Jacobian of output pose with respect to current
   * time offset [p, q]_6*[toff_MI]_1.
   * @return True if able to get the result.
   */
  bool GetPoseWithJacobian(
      const double &timestamp, Eigen::Vector3d *p_ptr,
      Eigen::Quaterniond *q_ptr, Eigen::Matrix<double, 6, 6> *cov_ptr,
      Eigen::Matrix<double, 6, 1> *J_pose_toff_ptr) override;

 private:
  /**
   * @brief Linear interpolation between two pose data to obtain the pose data
   * at the target timestamp, using spherical linear interpolation (SLERP) for
   * rotation and linear interpolation (LERP) for translation.
   *
   * @param target_t Target timestamp (s).
   * @param time_0 Timestamp of the first pose.
   * @param pose_0 SE(3) matrix of the first pose.
   * @param time_1 Timestamp of the second pose.
   * @param pose_1 SE(3) matrix of the second pose.
   * @param lambda_ptr Target time weight factor.
   * @param target_p_ptr Translation part of the target pose.
   * @param target_q_ptr Rotation part of the target pose.
   */
  void LinearInterpolatePose(const double &target_t, const double &time_0,
                             const Eigen::Matrix4d &pose_0,
                             const double &time_1,
                             const Eigen::Matrix4d &pose_1, double *lambda_ptr,
                             Eigen::Vector3d *target_p_ptr,
                             Eigen::Quaterniond *target_q_ptr);
};

}  // namespace mocap2gt
