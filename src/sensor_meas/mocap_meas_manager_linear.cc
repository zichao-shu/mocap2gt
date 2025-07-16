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

#include "mocap2gt/sensor_meas/mocap_meas_manager_linear.h"

namespace mocap2gt {

bool MoCapMeasManagerLinear::GetPose(const double &timestamp,
                                     Eigen::Vector3d *p_ptr,
                                     Eigen::Quaterniond *q_ptr) {
  // Set the default values.
  *p_ptr = Eigen::Vector3d::Zero();
  *q_ptr = Eigen::Quaterniond::Identity();

  if (traj_points_map_.size() < 2) {
    PRINT_ERROR("[Error]: Insufficient MoCap measurements. \n");

    std::exit(EXIT_FAILURE);
  }

  // Retrieve MoCap data at the target time.
  auto target_ub = traj_points_map_.upper_bound(timestamp);

  if (target_ub == traj_points_map_.begin() ||
      target_ub == traj_points_map_.end()) {
    PRINT_WARNING(
        "[Warning]: Specified pose interpolation time: %f is out of the range "
        "of the MoCap measurements: (%f, %f). \n",
        timestamp, traj_points_map_.begin()->first,
        traj_points_map_.rbegin()->first);

    return false;
  }

  auto data_0 = target_ub--;
  auto data_1 = target_ub;

  if (std::abs(timestamp - data_0->first) > 0.1 ||
      std::abs(timestamp - data_1->first) > 0.1) {
    PRINT_WARNING(
        "[Warning]: Missing MoCap data for more than 0.1s around %f, unable to "
        "perform an accurate interpolation. \n",
        timestamp);

    return false;
  }

  // Interpolation.
  double lambda = 0.;
  LinearInterpolatePose(timestamp, data_0->first, data_0->second, data_1->first,
                        data_1->second, &lambda, p_ptr, q_ptr);

  return true;
}

bool MoCapMeasManagerLinear::GetAngleVelocity(
    const double &timestamp, Eigen::Vector3d *angular_vel_ptr) {
  // Set the default values.
  *angular_vel_ptr = Eigen::Vector3d::Zero();

  constexpr double kCalVelHalfRange = 0.025;
  double time_before = timestamp - kCalVelHalfRange;
  double time_after = timestamp + kCalVelHalfRange;

  Eigen::Vector3d p_before = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_after = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_before = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_after = Eigen::Quaterniond::Identity();
  bool has_before = GetPose(time_before, &p_before, &q_before);
  bool has_after = GetPose(time_after, &p_after, &q_after);

  if (!has_before || !has_after) {
    PRINT_WARNING(
        "[Warning]: Specified velocity calculation timestamp is out of range. "
        "\n");

    return false;
  }

  Eigen::Quaterniond delta_q = q_before.inverse() * q_after;
  Eigen::AngleAxisd delta_aa(delta_q);
  *angular_vel_ptr = delta_aa.angle() * delta_aa.axis() / kCalVelHalfRange / 2;

  return true;
}

bool MoCapMeasManagerLinear::GetPoseWithJacobian(
    const double &timestamp, Eigen::Vector3d *p_ptr, Eigen::Quaterniond *q_ptr,
    Eigen::Matrix<double, 6, 6> *cov_ptr,
    Eigen::Matrix<double, 6, 1> *J_pose_toff_ptr) {
  // Set the default values.
  *p_ptr = Eigen::Vector3d::Zero();
  *q_ptr = Eigen::Quaterniond::Identity();
  *cov_ptr = Eigen::Matrix<double, 6, 6>::Zero();
  *J_pose_toff_ptr = Eigen::Matrix<double, 6, 1>::Zero();

  if (traj_points_map_.size() < 2) {
    PRINT_ERROR("[Error]: Insufficient MoCap measurements. \n");

    std::exit(EXIT_FAILURE);
  }

  // Retrieve MoCap data at the target time.
  auto target_ub = traj_points_map_.upper_bound(timestamp);

  if (target_ub == traj_points_map_.begin() ||
      target_ub == traj_points_map_.end()) {
    PRINT_WARNING(
        "[Warning]: Specified pose interpolation time: %f is out of the range "
        "of the MoCap measurements: (%f, %f). \n",
        timestamp, traj_points_map_.begin()->first,
        traj_points_map_.rbegin()->first);

    return false;
  }

  auto data_0 = target_ub--;
  auto data_1 = target_ub;

  if (std::abs(timestamp - data_0->first) > 0.1 ||
      std::abs(timestamp - data_1->first) > 0.1) {
    PRINT_WARNING(
        "[Warning]: Missing MoCap data for more than 0.1s around %f, unable to "
        "perform an accurate interpolation. \n",
        timestamp);

    return false;
  }

  // Interpolation.
  double lambda = 0.;
  LinearInterpolatePose(timestamp, data_0->first, data_0->second, data_1->first,
                        data_1->second, &lambda, p_ptr, q_ptr);

  // ===========================================================================

  // Here we simplify the covariance matrix of MoCap interpolation measurement,
  // assuming it to be constant at all times. For rigorous results, please refer
  // to the covariance matrix propagation principle.
  cov_ptr->block<3, 3>(0, 0) = trans_noise_cov_;
  cov_ptr->block<3, 3>(3, 3) = rot_noise_cov_;

  // ===========================================================================

  // Calculate the Jacobian of output pose with respect to time offset [p,
  // q]_6*[toff_MI]_1. Refer to formula (41)-(43) and (46).
  Eigen::Vector3d p_0(data_0->second.block<3, 1>(0, 3));
  Eigen::Vector3d p_1(data_1->second.block<3, 1>(0, 3));
  Eigen::Quaterniond q_0(data_0->second.block<3, 3>(0, 0));
  Eigen::Quaterniond q_1(data_1->second.block<3, 3>(0, 0));

  double J_lambda_toff = -1. / (data_1->first - data_0->first);
  Eigen::Vector3d omega =
      lambda * So3Log((q_0.inverse() * q_1).toRotationMatrix());
  Eigen::Vector3d J_p_lambda = p_1 - p_0;
  Eigen::Vector3d J_q_lambda =
      So3Jr(omega) * So3Log((q_0.inverse() * q_1).toRotationMatrix());

  J_pose_toff_ptr->setZero();
  J_pose_toff_ptr->block<3, 1>(0, 0) = J_p_lambda * J_lambda_toff;
  J_pose_toff_ptr->block<3, 1>(3, 0) = J_q_lambda * J_lambda_toff;

  // ===========================================================================

  return true;
}

void MoCapMeasManagerLinear::LinearInterpolatePose(
    const double &target_t, const double &time_0, const Eigen::Matrix4d &pose_0,
    const double &time_1, const Eigen::Matrix4d &pose_1, double *lambda_ptr,
    Eigen::Vector3d *target_p_ptr, Eigen::Quaterniond *target_q_ptr) {
  // Time-distance lambda.
  *lambda_ptr = (target_t - time_0) / (time_1 - time_0);

  Eigen::Vector3d p_0(pose_0.block<3, 1>(0, 3));
  Eigen::Vector3d p_1(pose_1.block<3, 1>(0, 3));
  Eigen::Quaterniond q_0(pose_0.block<3, 3>(0, 0));
  Eigen::Quaterniond q_1(pose_1.block<3, 3>(0, 0));

  // The translation part is interpolated using LERP.
  *target_p_ptr = (1 - *lambda_ptr) * p_0 + (*lambda_ptr) * p_1;
  // The rotation component is interpolated using SLERP. Refer to formula (33).
  *target_q_ptr = q_0.slerp(*lambda_ptr, q_1).normalized();
}

}  // namespace mocap2gt
