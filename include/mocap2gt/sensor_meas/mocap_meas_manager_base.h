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

#include <functional>
#include <map>
#include <utility>

#include "mocap2gt/utils/geometry_utils.h"
#include "mocap2gt/utils/printer.h"

namespace mocap2gt {

/**
 * @brief Base class for storing and managing MoCap data.
 */
class MoCapMeasManagerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Type definition of aligned timestamped SE(3) map.
  typedef std::map<
      double, Eigen::Matrix4d, std::less<double>,
      Eigen::aligned_allocator<std::pair<const double, Eigen::Matrix4d>>>
      TimestampSE3Map;

  /**
   * @brief Construct the MoCap measurement manager, initialize the MoCap noise
   * covariance matrix.
   *
   * @param trans_noise_v Standard deviation vector of MoCap translation noise.
   * @param rot_noise_v Standard deviation vector of MoCap rotation noise.
   */
  MoCapMeasManagerBase(const Eigen::Vector3d &trans_noise_v,
                       const Eigen::Vector3d &rot_noise_v)
      : trans_noise_cov_(trans_noise_v.array().square().matrix().asDiagonal()),
        rot_noise_cov_(rot_noise_v.array().square().matrix().asDiagonal()) {}

  /**
   * @brief Feed MoCap pose measurement, convert to timestamped SE(3) and save
   * in the map.
   *
   * @param timestamp Timestamp of the pose (s).
   * @param p Translation part (p_WM) of the pose.
   * @param q Rotation part (q_WM) of the pose.
   * @return True if we can feed it.
   */
  bool FeedPose(const double &timestamp, const Eigen::Vector3d &p,
                const Eigen::Quaterniond &q);

  /**
   * @brief Generate uniform control points. For B-spline measurement only.
   *
   * @param min_control_dt Minimum time interval of control points.
   */
  virtual void GenerateControlPoints(const double &min_control_dt = 0.01) {}

  /**
   * @brief Interpolate to obtain the MoCap measurement at the given time.
   *
   * @param timestamp Target timestamp (s).
   * @param p_ptr Translation part (p_WM) of the target pose.
   * @param q_ptr Rotation part (q_WM) of the target pose.
   * @return True if able to get the pose.
   */
  virtual bool GetPose(const double &timestamp, Eigen::Vector3d *p_ptr,
                       Eigen::Quaterniond *q_ptr) = 0;

  /**
   * @brief Get the angular velocity at the given timestamp.
   *
   * @param timestamp Target timestamp (s).
   * @param angular_vel_ptr Target angular velocity in the local frame.
   * @return True if able to get the angular velocity.
   */
  virtual bool GetAngleVelocity(const double &timestamp,
                                Eigen::Vector3d *angular_vel_ptr) = 0;

  /**
   * @brief Interpolate to obtain the MoCap measurement, covariance and the
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
  virtual bool GetPoseWithJacobian(
      const double &timestamp, Eigen::Vector3d *p_ptr,
      Eigen::Quaterniond *q_ptr, Eigen::Matrix<double, 6, 6> *cov_ptr,
      Eigen::Matrix<double, 6, 1> *J_pose_toff_ptr) = 0;

  // Accessors for MoCap data information.
  const TimestampSE3Map &get_traj_points_map() const {
    return traj_points_map_;
  }
  const double &get_start_time() const { return start_time_; }
  const double &get_end_time() const { return end_time_; }
  const int get_data_count() const { return traj_points_map_.size(); }

 protected:
  // Timestamped SE(3) poses of the original trajectory (timestamp, T_WI).
  TimestampSE3Map traj_points_map_;

  // Motion capture noise covariance matrix.
  const Eigen::Matrix3d trans_noise_cov_;
  const Eigen::Matrix3d rot_noise_cov_;

  double start_time_ = -1.;
  double end_time_ = -1.;
};

}  // namespace mocap2gt
