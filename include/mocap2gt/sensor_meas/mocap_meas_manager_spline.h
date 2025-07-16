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
#include "mocap2gt/utils/geometry_utils.h"

namespace mocap2gt {

/**
 * @brief Derived class for MoCap data management, utilizing cubic SE(3)
 * B-spline pose interpolation. One can obtain the MoCap measurement, Jacobian,
 * covariance and the closed-form inertial data at the given time.
 *
 * The use of b-splines for interpolation has the following properties:
 * - Local control, allowing the system to function online as well as in batch;
 * - C^2-continuity to enable inertial predictions and calculations;
 * - Good approximation of minimal torque trajectories;
 * - A parameterization of rigid-body motion devoid of singularities.
 *
 * For more details about cubic SE(3) B-spline pose interpolation and Jacobian
 * calculation, please refer to:
 * "Research Notes and Jacobians: Continuous-Time Visual-Inertial Trajectory
 * Estimation with Event Cameras"
 * https://pgeneva.com/downloads/notes/2018_notes_mueffler2017arxiv.pdf
 */
class MoCapMeasManagerSpline : public MoCapMeasManagerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct the MoCap measurement manager, initialize the MoCap noise
   * covariance matrix.
   *
   * @param trans_noise_v Standard deviation vector of MoCap translation noise.
   * @param rot_noise_v Standard deviation vector of MoCap rotation noise.
   */
  MoCapMeasManagerSpline(const Eigen::Vector3d &trans_noise_v,
                         const Eigen::Vector3d &rot_noise_v)
      : MoCapMeasManagerBase(trans_noise_v, rot_noise_v) {}

  /**
   * @brief Generate B-spline uniform control points from the current SE(3)
   * poses. Always generate control points before interpolation.
   *
   * @param min_control_dt Minimum time interval (s) of control points.
   */
  void GenerateControlPoints(const double &min_control_dt = 0.01) override;

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

  /**
   * @brief Get the pose and inertial measurement from B-spline at a given
   * timestamp.
   *
   * @param timestamp Target timestamp (s).
   * @param p_ptr Translation part (p_WM) of the target pose.
   * @param q_ptr Rotation part (q_WM) of the target pose.
   * @param acc_ptr Target Acceleration in the local frame without
   * gravity measurement.
   * @param gyr_ptr Target angular velocity in the local frame.
   * @return True if we can get the result.
   */
  bool GetPoseInertialData(double timestamp, Eigen::Vector3d *p_ptr,
                           Eigen::Quaterniond *q_ptr, Eigen::Vector3d *acc_ptr,
                           Eigen::Vector3d *gyr_ptr);

 private:
  /**
   * @brief Find the two bounding poses for a given timestamp from a timestamped
   * poses map.
   *
   * @param timestamp Target timestamp (s).
   * @param poses Map of timestamped poses.
   * @param time_0_ptr Timestamp of the first pose.
   * @param pose_0_ptr SE(3) matrix of the first pose.
   * @param time_1_ptr Timestamp of the second pose.
   * @param pose_1_ptr SE(3) matrix of the second pose.
   * @return True if we can find bounding poses.
   */
  bool FindBoundingPoses(const double timestamp, const TimestampSE3Map &poses,
                         double *time_0_ptr, Eigen::Matrix4d *pose_0_ptr,
                         double *time_1_ptr, Eigen::Matrix4d *pose_1_ptr);

  /**
   * @brief Find two older poses and two newer poses (control points) for a
   * given timestamp.
   *
   * Note that we use a cubic B-spline, so that the pose at any time will be
   * controlled by four neighboring control points.
   *
   * @param timestamp Target timestamp (s).
   * @param time_0_ptr Timestamp of the first pose.
   * @param pose_0_ptr SE(3) matrix of the first pose.
   * @param time_1_ptr Timestamp of the second pose.
   * @param pose_1_ptr SE(3) matrix of the second pose.
   * @param time_2_ptr Timestamp of the third pose.
   * @param pose_2_ptr SE(3) matrix of the third pose.
   * @param time_3_ptr Timestamp of the fourth pose.
   * @param pose_3_ptr SE(3) matrix of the fourth pose.
   * @return True if we can find the four control poses.
   */
  bool FindControlPoints(const double timestamp, double *time_0_ptr,
                         Eigen::Matrix4d *pose_0_ptr, double *time_1_ptr,
                         Eigen::Matrix4d *pose_1_ptr, double *time_2_ptr,
                         Eigen::Matrix4d *pose_2_ptr, double *time_3_ptr,
                         Eigen::Matrix4d *pose_3_ptr);

  // Timestamped SE(3) control poses (timestamp, T_WI).
  TimestampSE3Map control_points_map_;

  // Uniform sampling time for control points.
  double control_dt_ = -1.;
};

}  // namespace mocap2gt
