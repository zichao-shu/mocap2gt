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

#include "mocap2gt/ceres_factor/mocap_interpolation_variable_factor.h"

namespace mocap2gt {

bool MoCapInterpolationVariableFactor::Evaluate(double const *const *parameters,
                                                double *residuals,
                                                double **jacobians) const {
  // Step 1: Get parameter from memory.
  // IMU state at timestamp i.
  const Eigen::Vector3d p_WI_i(parameters[0][0], parameters[0][1],
                               parameters[0][2]);
  const Eigen::Quaterniond q_WI_i(parameters[0][6], parameters[0][3],
                                  parameters[0][4], parameters[0][5]);

  // Spatial hand-eye calibration parameter.
  const Eigen::Vector3d p_MI(parameters[1][0], parameters[1][1],
                             parameters[1][2]);
  const Eigen::Quaterniond q_MI(parameters[1][6], parameters[1][3],
                                parameters[1][4], parameters[1][5]);

  // Time offset of the two control points adjacent to the current state.
  const double toff_MI_m = parameters[2][0] + toff_truncate_;
  const double toff_MI_n = parameters[3][0] + toff_truncate_;
  // Interpolate to calculate time offset of the current state.
  const double lambda =
      (t_I_i_ - t_control_point_m_) / (t_control_point_n_ - t_control_point_m_);
  const double toff_MI_current = (1 - lambda) * toff_MI_m + lambda * toff_MI_n;
  // Timestamp of MoCap measurement.
  const double t_M_i = t_I_i_ - toff_MI_current;

  // ============================================================================

  // Step 2: Calculate the residual.
  Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);

  Eigen::Vector3d p_WM_i = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_WM_i = Eigen::Quaterniond::Identity();
  Eigen::Matrix<double, 6, 6> meas_cov = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> d_interp_toff =
      Eigen::Matrix<double, 6, 1>::Zero();
  // Interpolation to obtain the measurement, covariance and the Jacobian.
  if (!mocap_manager_->GetPoseWithJacobian(t_M_i, &p_WM_i, &q_WM_i, &meas_cov,
                                           &d_interp_toff)) {
    PRINT_ERROR(
        "[Error]: Fail to evaluate the MoCap factor, with IMU timestamps and "
        "time offset of %f and %f. \n",
        t_I_i_, toff_MI_current);
    PRINT_ERROR(
        "[Error]: This may be caused by the current time offset exceeding the "
        "max time offset. Please provide a more accurate "
        "initial time offset or increase the max time offset. \n");

    return false;
  }

  // Compute the residual.
  residual.block<3, 1>(0, 0) = p_WI_i - q_WI_i * q_MI.inverse() * p_MI - p_WM_i;
  residual.block<3, 1>(3, 0) =
      2 * (q_WI_i * q_MI.inverse() * q_WM_i.inverse()).vec();

  if (std::isnan(meas_cov.norm()) || std::isnan(meas_cov.inverse().norm())) {
    PRINT_ERROR(
        "[Error]: MoCap interpolation covariance exception, with invalid norm. "
        "\n");

    return false;
  }
  // Combine the infomation matrix.
  Eigen::Matrix<double, 6, 6> meas_info_sqrt =
      Eigen::LLT<Eigen::Matrix<double, 6, 6>>(meas_cov.inverse())
          .matrixL()
          .transpose();
  residual = meas_info_sqrt * residual;

  // ============================================================================

  if (jacobians) {
    // Step 3: Calculate the Jacobian of the residual with respect to the IMU
    // pose at time i.
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_i(
          jacobians[0]);
      jacobian_pose_i.setZero();
      jacobian_pose_i.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      jacobian_pose_i.block<3, 3>(0, 3) =
          q_WI_i.toRotationMatrix() * Skew(q_MI.inverse() * p_MI);
      jacobian_pose_i.block<3, 3>(3, 3) =
          (QuatLeftProd(q_WI_i) *
           QuatRightProd(q_MI.inverse() * q_WM_i.inverse()))
              .bottomRightCorner<3, 3>();

      // Combine the infomation matrix.
      jacobian_pose_i = meas_info_sqrt * jacobian_pose_i;
    }

    // ============================================================================

    // Step 4: Calculate the Jacobian of the residual with respect to the
    // spatial hand-eye calibration parameter (extrinsic).
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>>
          jacobian_extrinsic(jacobians[1]);
      jacobian_extrinsic.setZero();
      jacobian_extrinsic.block<3, 3>(0, 0) =
          -(q_WI_i * q_MI.inverse()).toRotationMatrix();
      jacobian_extrinsic.block<3, 3>(0, 3) =
          -q_WI_i.toRotationMatrix() * Skew(q_MI.inverse() * p_MI);
      jacobian_extrinsic.block<3, 3>(3, 3) =
          -(QuatLeftProd(q_WM_i * q_MI) * QuatRightProd(q_WI_i.inverse()))
               .bottomRightCorner<3, 3>();

      // Combine the infomation matrix.
      jacobian_extrinsic = meas_info_sqrt * jacobian_extrinsic;
    }

    // ============================================================================

    // Step 5: Calculate the Jacobian of the residual with respect to the
    // temporal hand-eye calibration parameter (time offset).
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 6, 2>> jacobian_toff_current(
          jacobians[2]);
      jacobian_toff_current.setZero();
      Eigen::Matrix<double, 6, 6> jacobian_interp =
          Eigen::Matrix<double, 6, 6>::Zero();
      jacobian_interp.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
      jacobian_interp.block<3, 3>(3, 3) =
          -(QuatLeftProd(q_WM_i) * QuatRightProd(q_MI * q_WI_i.inverse()))
               .bottomRightCorner<3, 3>();

      Eigen::Matrix<double, 1, 2> jacobian_toff_mn;
      jacobian_toff_mn.setZero();
      jacobian_toff_mn << (1. - lambda), lambda;

      jacobian_toff_current =
          jacobian_interp * d_interp_toff * jacobian_toff_mn;

      // Combine the infomation matrix.
      jacobian_toff_current = meas_info_sqrt * jacobian_toff_current;
    }

    // ============================================================================
  }
  return true;
}

}  // namespace mocap2gt
