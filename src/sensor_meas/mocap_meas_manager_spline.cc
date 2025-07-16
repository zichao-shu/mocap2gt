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

#include "mocap2gt/sensor_meas/mocap_meas_manager_spline.h"

namespace mocap2gt {

void MoCapMeasManagerSpline::GenerateControlPoints(
    const double &min_control_dt) {
  // Determine the control point time interval for our uniform B-spline.
  double dt_sum =
      traj_points_map_.rbegin()->first - traj_points_map_.begin()->first;
  control_dt_ = dt_sum / (traj_points_map_.size() - 1);
  control_dt_ = (control_dt_ < min_control_dt) ? min_control_dt : control_dt_;
  PRINT_INFO(
      "[Info]: B-spline control point dt = %.3f (original dt of %.3f). \n",
      control_dt_, dt_sum / (traj_points_map_.size() - 1));

  // Interpolation to generate control points.
  double current_time = traj_points_map_.begin()->first;
  while (true) {
    double time_0, time_1;
    Eigen::Matrix4d pose_0, pose_1;
    if (!FindBoundingPoses(current_time, traj_points_map_, &time_0, &pose_0,
                           &time_1, &pose_1)) {
      break;
    }

    // Interpolation on SE(3) manifold.
    double lambda = (current_time - time_0) / (time_1 - time_0);
    Eigen::Matrix4d pose_interp =
        pose_0 * Se3Exp(lambda * Se3Log(pose_0.inverse() * pose_1));
    control_points_map_.insert({current_time, pose_interp});
    current_time += control_dt_;
  }

  if (control_points_map_.size() <= 4) {
    PRINT_ERROR("[Error]: Insufficient control points: %d \n",
                control_points_map_.size());

    std::exit(EXIT_FAILURE);
  }

  // Update the start/end time, from the second control point to the
  // second-to-last control point.
  start_time_ = std::next(control_points_map_.begin())->first;
  end_time_ = std::next(control_points_map_.rbegin())->first;

  PRINT_INFO("[Info]: Control points generation done. \n")
  PRINT_INFO("[Info]: Start/end time of the B-spline: %f/%f \n", start_time_,
             end_time_);
}

bool MoCapMeasManagerSpline::GetPose(const double &timestamp,
                                     Eigen::Vector3d *p_ptr,
                                     Eigen::Quaterniond *q_ptr) {
  // Set the default values.
  *p_ptr = Eigen::Vector3d::Zero();
  *q_ptr = Eigen::Quaterniond::Identity();

  if (control_points_map_.size() <= 4) {
    PRINT_ERROR("[Error]: Insufficient control points: %d. \n",
                control_points_map_.size());

    std::exit(EXIT_FAILURE);
  }

  // Get the control points for the target timestamp.
  double time_0, time_1, time_2, time_3;
  Eigen::Matrix4d pose_0, pose_1, pose_2, pose_3;
  if (!FindControlPoints(timestamp, &time_0, &pose_0, &time_1, &pose_1, &time_2,
                         &pose_2, &time_3, &pose_3)) {
    return false;
  }

  // De Boor-Cox matrix scalars. Refer to formula (47)-(49).
  double u = (timestamp - time_1) / control_dt_;
  double B_0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double B_1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double B_2 = 1.0 / 6.0 * (u * u * u);

  // Adjacent control point differences in tangent space.
  Eigen::Matrix<double, 6, 1> omega_01 = Se3Log(pose_0.inverse() * pose_1);
  Eigen::Matrix<double, 6, 1> omega_12 = Se3Log(pose_1.inverse() * pose_2);
  Eigen::Matrix<double, 6, 1> omega_23 = Se3Log(pose_2.inverse() * pose_3);

  // A0~A2, Refer to formula (44).
  Eigen::Matrix4d A_0 = Se3Exp(B_0 * omega_01);
  Eigen::Matrix4d A_1 = Se3Exp(B_1 * omega_12);
  Eigen::Matrix4d A_2 = Se3Exp(B_2 * omega_23);

  // Get the interpolated pose. Refer to formula (40).
  Eigen::Matrix4d pose_interp = pose_0 * A_0 * A_1 * A_2;
  *p_ptr = pose_interp.block<3, 1>(0, 3);
  *q_ptr = Eigen::Quaterniond(pose_interp.block<3, 3>(0, 0));

  return true;
}

bool MoCapMeasManagerSpline::GetAngleVelocity(
    const double &timestamp, Eigen::Vector3d *angular_vel_ptr) {
  // Set the default values.
  *angular_vel_ptr = Eigen::Vector3d::Zero();

  if (control_points_map_.size() <= 4) {
    PRINT_ERROR("[Error]: Insufficient control points: %d. \n",
                control_points_map_.size());

    std::exit(EXIT_FAILURE);
  }

  // Get the control points for the target timestamp.
  double time_0, time_1, time_2, time_3;
  Eigen::Matrix4d pose_0, pose_1, pose_2, pose_3;
  if (!FindControlPoints(timestamp, &time_0, &pose_0, &time_1, &pose_1, &time_2,
                         &pose_2, &time_3, &pose_3)) {
    return false;
  }

  // De Boor-Cox matrix scalars. Refer to formula (47)-(49).
  double u = (timestamp - time_1) / control_dt_;
  double B_0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double B_1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double B_2 = 1.0 / 6.0 * (u * u * u);
  // The first derivative with respect to time. Refer to formula (50)-(52).
  double B_0_dot = 1.0 / (6.0 * control_dt_) * (3 - 6 * u + 3 * u * u);
  double B_1_dot = 1.0 / (6.0 * control_dt_) * (3 + 6 * u - 6 * u * u);
  double B_2_dot = 1.0 / (6.0 * control_dt_) * (3 * u * u);

  // Adjacent control point differences in tangent space.
  Eigen::Matrix<double, 6, 1> omega_01 = Se3Log(pose_0.inverse() * pose_1);
  Eigen::Matrix<double, 6, 1> omega_12 = Se3Log(pose_1.inverse() * pose_2);
  Eigen::Matrix<double, 6, 1> omega_23 = Se3Log(pose_2.inverse() * pose_3);

  // A0~A2 and the derivatives with respect to time. Refer to formula (44).
  Eigen::Matrix4d A_0 = Se3Exp(B_0 * omega_01);
  Eigen::Matrix4d A_1 = Se3Exp(B_1 * omega_12);
  Eigen::Matrix4d A_2 = Se3Exp(B_2 * omega_23);
  // The first derivative with respect to time. Refer to formula (45).
  Eigen::Matrix4d A_0_dot = B_0_dot * Se3Skew(omega_01) * A_0;
  Eigen::Matrix4d A_1_dot = B_1_dot * Se3Skew(omega_12) * A_1;
  Eigen::Matrix4d A_2_dot = B_2_dot * Se3Skew(omega_23) * A_2;

  // Get the interpolated pose. Refer to formula (40).
  Eigen::Matrix4d pose_interp = pose_0 * A_0 * A_1 * A_2;
  Eigen::Quaterniond q_interp =
      Eigen::Quaterniond(pose_interp.block<3, 3>(0, 0));

  // Get the interpolated angular velocity. Refer to formula (41) and (26).
  Eigen::Matrix4d vel_interp =
      pose_0 *
      (A_0_dot * A_1 * A_2 + A_0 * A_1_dot * A_2 + A_0 * A_1 * A_2_dot);
  *angular_vel_ptr = Vee(q_interp.inverse() * vel_interp.block<3, 3>(0, 0));

  return true;
}

bool MoCapMeasManagerSpline::GetPoseWithJacobian(
    const double &timestamp, Eigen::Vector3d *p_ptr, Eigen::Quaterniond *q_ptr,
    Eigen::Matrix<double, 6, 6> *cov_ptr,
    Eigen::Matrix<double, 6, 1> *J_pose_toff_ptr) {
  // Set the default values.
  *p_ptr = Eigen::Vector3d::Zero();
  *q_ptr = Eigen::Quaterniond::Identity();
  *cov_ptr = Eigen::Matrix<double, 6, 6>::Zero();
  *J_pose_toff_ptr = Eigen::Matrix<double, 6, 1>::Zero();

  if (control_points_map_.size() <= 4) {
    PRINT_ERROR("[Error]: Insufficient control points: %d. \n",
                control_points_map_.size());

    std::exit(EXIT_FAILURE);
  }

  // Get the control points for the target timestamp.
  double time_0, time_1, time_2, time_3;
  Eigen::Matrix4d pose_0, pose_1, pose_2, pose_3;
  if (!FindControlPoints(timestamp, &time_0, &pose_0, &time_1, &pose_1, &time_2,
                         &pose_2, &time_3, &pose_3)) {
    return false;
  }

  // De Boor-Cox matrix scalars. Refer to formula (47)-(49).
  double u = (timestamp - time_1) / control_dt_;
  double B_0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double B_1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double B_2 = 1.0 / 6.0 * (u * u * u);
  // The first derivative with respect to time. Refer to formula (50)-(52).
  double B_0_dot = 1.0 / (6.0 * control_dt_) * (3 - 6 * u + 3 * u * u);
  double B_1_dot = 1.0 / (6.0 * control_dt_) * (3 + 6 * u - 6 * u * u);
  double B_2_dot = 1.0 / (6.0 * control_dt_) * (3 * u * u);

  // Adjacent control point differences in tangent space.
  Eigen::Matrix<double, 6, 1> omega_01 = Se3Log(pose_0.inverse() * pose_1);
  Eigen::Matrix<double, 6, 1> omega_12 = Se3Log(pose_1.inverse() * pose_2);
  Eigen::Matrix<double, 6, 1> omega_23 = Se3Log(pose_2.inverse() * pose_3);

  // A0~A2 and the derivatives with respect to time. Refer to formula (44).
  Eigen::Matrix4d A_0 = Se3Exp(B_0 * omega_01);
  Eigen::Matrix4d A_1 = Se3Exp(B_1 * omega_12);
  Eigen::Matrix4d A_2 = Se3Exp(B_2 * omega_23);
  // The first derivative with respect to time. Refer to formula (45).
  Eigen::Matrix4d A_0_dot = B_0_dot * Se3Skew(omega_01) * A_0;
  Eigen::Matrix4d A_1_dot = B_1_dot * Se3Skew(omega_12) * A_1;
  Eigen::Matrix4d A_2_dot = B_2_dot * Se3Skew(omega_23) * A_2;

  // Get the interpolated pose. Refer to formula (40).
  Eigen::Matrix4d pose_interp = pose_0 * A_0 * A_1 * A_2;
  *p_ptr = pose_interp.block<3, 1>(0, 3);
  *q_ptr = Eigen::Quaterniond(pose_interp.block<3, 3>(0, 0));

  // Get the interpolated velocity (The derivative with respect to time). Refer
  // to formula (41).
  Eigen::Matrix4d vel_interp =
      pose_0 *
      (A_0_dot * A_1 * A_2 + A_0 * A_1_dot * A_2 + A_0 * A_1 * A_2_dot);
  Eigen::Vector3d J_p_t = vel_interp.block<3, 1>(0, 3);
  Eigen::Vector3d J_q_t = Vee(q_ptr->inverse() * vel_interp.block<3, 3>(0, 0));

  // Derivatives with respect to time offset (multiply by -1).
  J_pose_toff_ptr->block<3, 1>(0, 0) = -J_p_t;
  J_pose_toff_ptr->block<3, 1>(3, 0) = -J_q_t;

  cov_ptr->block<3, 3>(0, 0) = trans_noise_cov_;
  cov_ptr->block<3, 3>(3, 3) = rot_noise_cov_;

  return true;
}

bool MoCapMeasManagerSpline::GetPoseInertialData(double timestamp,
                                                 Eigen::Vector3d *p_ptr,
                                                 Eigen::Quaterniond *q_ptr,
                                                 Eigen::Vector3d *acc_ptr,
                                                 Eigen::Vector3d *gyr_ptr) {
  // Set the default values.
  *p_ptr = Eigen::Vector3d::Zero();
  *q_ptr = Eigen::Quaterniond::Identity();
  *acc_ptr = Eigen::Vector3d::Zero();
  *gyr_ptr = Eigen::Vector3d::Zero();

  if (control_points_map_.size() <= 4) {
    PRINT_ERROR("[Error]: Insufficient control points: %d. \n",
                control_points_map_.size());

    std::exit(EXIT_FAILURE);
  }

  // Get the control points for the target timestamp.
  double time_0, time_1, time_2, time_3;
  Eigen::Matrix4d pose_0, pose_1, pose_2, pose_3;
  if (!FindControlPoints(timestamp, &time_0, &pose_0, &time_1, &pose_1, &time_2,
                         &pose_2, &time_3, &pose_3)) {
    return false;
  }

  // De Boor-Cox matrix scalars. Refer to formula (47)-(49).
  double u = (timestamp - time_1) / control_dt_;
  double B_0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double B_1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double B_2 = 1.0 / 6.0 * (u * u * u);
  // The first derivative with respect to time. Refer to formula (50)-(52).
  double B_0_dot = 1.0 / (6.0 * control_dt_) * (3 - 6 * u + 3 * u * u);
  double B_1_dot = 1.0 / (6.0 * control_dt_) * (3 + 6 * u - 6 * u * u);
  double B_2_dot = 1.0 / (6.0 * control_dt_) * (3 * u * u);
  // The second derivative with respect to time. Refer to formula (53)-(55).
  double B_0_ddot = 1.0 / (6.0 * control_dt_ * control_dt_) * (-6 + 6 * u);
  double B_1_ddot = 1.0 / (6.0 * control_dt_ * control_dt_) * (6 - 12 * u);
  double B_2_ddot = 1.0 / (6.0 * control_dt_ * control_dt_) * (6 * u);

  // Adjacent control point differences in tangent space.
  Eigen::Matrix<double, 6, 1> omega_01 = Se3Log(pose_0.inverse() * pose_1);
  Eigen::Matrix<double, 6, 1> omega_12 = Se3Log(pose_1.inverse() * pose_2);
  Eigen::Matrix<double, 6, 1> omega_23 = Se3Log(pose_2.inverse() * pose_3);
  // Skew-symmetric matrix.
  Eigen::Matrix4d omega_01_skew = Se3Skew(omega_01);
  Eigen::Matrix4d omega_12_skew = Se3Skew(omega_12);
  Eigen::Matrix4d omega_23_skew = Se3Skew(omega_23);

  // A0~A2 and the derivatives with respect to time. Refer to formula (44).
  Eigen::Matrix4d A_0 = Se3Exp(B_0 * omega_01);
  Eigen::Matrix4d A_1 = Se3Exp(B_1 * omega_12);
  Eigen::Matrix4d A_2 = Se3Exp(B_2 * omega_23);
  // The first derivative with respect to time. Refer to formula (45).
  Eigen::Matrix4d A_0_dot = B_0_dot * Se3Skew(omega_01) * A_0;
  Eigen::Matrix4d A_1_dot = B_1_dot * Se3Skew(omega_12) * A_1;
  Eigen::Matrix4d A_2_dot = B_2_dot * Se3Skew(omega_23) * A_2;
  // The second derivative with respect to time. Refer to formula (46).
  Eigen::Matrix4d A_0_ddot =
      B_0_dot * omega_01_skew * A_0_dot + B_0_ddot * omega_01_skew * A_0;
  Eigen::Matrix4d A_1_ddot =
      B_1_dot * omega_12_skew * A_1_dot + B_1_ddot * omega_12_skew * A_1;
  Eigen::Matrix4d A_2_ddot =
      B_2_dot * omega_23_skew * A_2_dot + B_2_ddot * omega_23_skew * A_2;

  // Get the interpolated pose. Refer to formula (40).
  Eigen::Matrix4d pose_interp = pose_0 * A_0 * A_1 * A_2;
  *p_ptr = pose_interp.block<3, 1>(0, 3);
  *q_ptr = Eigen::Quaterniond(pose_interp.block<3, 3>(0, 0));

  // Get the interpolated angular velocity. Refer to formula (41) and (26).
  Eigen::Matrix4d vel_interp =
      pose_0 *
      (A_0_dot * A_1 * A_2 + A_0 * A_1_dot * A_2 + A_0 * A_1 * A_2_dot);
  *gyr_ptr = Vee(q_ptr->inverse() * vel_interp.block<3, 3>(0, 0));

  // Get the interpolated acceleration. Refer to formula (42) and (27).
  Eigen::Matrix4d acc_interp =
      pose_0 * (A_0_ddot * A_1 * A_2 + A_0 * A_1_ddot * A_2 +
                A_0 * A_1 * A_2_ddot + 2 * A_0_dot * A_1_dot * A_2 +
                2 * A_0 * A_1_dot * A_2_dot + 2 * A_0_dot * A_1 * A_2_dot);
  *acc_ptr = q_ptr->inverse() * acc_interp.block<3, 1>(0, 3);

  return true;
}

bool MoCapMeasManagerSpline::FindBoundingPoses(const double timestamp,
                                               const TimestampSE3Map &poses,
                                               double *time_0_ptr,
                                               Eigen::Matrix4d *pose_0_ptr,
                                               double *time_1_ptr,
                                               Eigen::Matrix4d *pose_1_ptr) {
  // Set the default values.
  *time_0_ptr = -1.;
  *time_1_ptr = -1.;
  *pose_0_ptr = Eigen::Matrix4d::Identity();
  *pose_1_ptr = Eigen::Matrix4d::Identity();

  if (poses.size() < 2) {
    PRINT_WARNING("[Warning]: Insufficient data to find bounding poses. \n");

    return false;
  }

  // Find the upper bound and the previous member.
  auto target_ub = poses.upper_bound(timestamp);
  if (target_ub == poses.begin() || target_ub == poses.end()) {
    return false;
  }

  auto data_iter_0 = std::prev(target_ub, 1);
  auto data_iter_1 = target_ub;

  *time_0_ptr = data_iter_0->first;
  *pose_0_ptr = data_iter_0->second;
  *time_1_ptr = data_iter_1->first;
  *pose_1_ptr = data_iter_1->second;

  return true;
}

bool MoCapMeasManagerSpline::FindControlPoints(
    const double timestamp, double *time_0_ptr, Eigen::Matrix4d *pose_0_ptr,
    double *time_1_ptr, Eigen::Matrix4d *pose_1_ptr, double *time_2_ptr,
    Eigen::Matrix4d *pose_2_ptr, double *time_3_ptr,
    Eigen::Matrix4d *pose_3_ptr) {
  // Set the default values.
  *time_0_ptr = -1;
  *time_1_ptr = -1;
  *time_2_ptr = -1;
  *time_3_ptr = -1;
  *pose_0_ptr = Eigen::Matrix4d::Identity();
  *pose_1_ptr = Eigen::Matrix4d::Identity();
  *pose_2_ptr = Eigen::Matrix4d::Identity();
  *pose_3_ptr = Eigen::Matrix4d::Identity();

  // Assert the target time is within the valid range to find four corresponding
  // control points.
  if (timestamp < start_time_ || timestamp > end_time_) {
    PRINT_WARNING(
        "[Warning]: Failed to find B-spline control points. Specified "
        "timestamp: %f is out of the range of the B-spline control "
        "points: (%f, %f). \n",
        timestamp, start_time_, end_time_);

    return false;
  }
  // Find the two bounding poses for the given timestamp.
  if (!FindBoundingPoses(timestamp, control_points_map_, time_1_ptr, pose_1_ptr,
                         time_2_ptr, pose_2_ptr)) {
    return false;
  }

  // Find the poses that are below and above.
  auto data_iter_1 = control_points_map_.find(*time_1_ptr);
  auto data_iter_2 = control_points_map_.find(*time_2_ptr);
  auto data_iter_0 = --data_iter_1;
  auto data_iter_3 = ++data_iter_2;

  // Set the oldest one
  *time_0_ptr = data_iter_0->first;
  *pose_0_ptr = data_iter_0->second;

  // Set the newest one
  *time_3_ptr = data_iter_3->first;
  *pose_3_ptr = data_iter_3->second;

  return true;
}

}  // namespace mocap2gt
