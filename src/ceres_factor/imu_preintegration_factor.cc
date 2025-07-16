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

#include "mocap2gt/ceres_factor/imu_preintegration_factor.h"

namespace mocap2gt {

bool IMUPreintegrationFactor::Evaluate(double const *const *parameters,
                                       double *residuals,
                                       double **jacobians) const {
  // Step 1: Get parameter from memory.
  // IMU state at timestamp i.
  const Eigen::Vector3d p_WI_i(parameters[0][0], parameters[0][1],
                               parameters[0][2]);
  const Eigen::Quaterniond q_WI_i(parameters[0][6], parameters[0][3],
                                  parameters[0][4], parameters[0][5]);
  const Eigen::Vector3d v_WI_i(parameters[1][0], parameters[1][1],
                               parameters[1][2]);
  const Eigen::Vector3d ba_i(parameters[1][3], parameters[1][4],
                             parameters[1][5]);
  const Eigen::Vector3d bg_i(parameters[1][6], parameters[1][7],
                             parameters[1][8]);

  // IMU state at timestamp j.
  const Eigen::Vector3d p_WI_j(parameters[2][0], parameters[2][1],
                               parameters[2][2]);
  const Eigen::Quaterniond q_WI_j(parameters[2][6], parameters[2][3],
                                  parameters[2][4], parameters[2][5]);
  const Eigen::Vector3d v_WI_j(parameters[3][0], parameters[3][1],
                               parameters[3][2]);
  const Eigen::Vector3d ba_j(parameters[3][3], parameters[3][4],
                             parameters[3][5]);
  const Eigen::Vector3d bg_j(parameters[3][6], parameters[3][7],
                             parameters[3][8]);

  // Gravity aligned rotation.
  const double roll = parameters[4][0];
  const double pitch = parameters[4][1];
  const Eigen::Vector3d rpy_WG(roll, pitch, 0.);
  const Eigen::Quaterniond q_WG = Rpy2Quat(rpy_WG);

  // ===========================================================================

  // Step 2: Calculate the residual.
  Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);

  if (preintegrator_->get_state_jacobian().maxCoeff() > 1e9 ||
      preintegrator_->get_state_jacobian().minCoeff() < -1e9) {
    PRINT_WARNING(
        "[Warning]: Numerical unstable in IMU preintegration state Jacobian, "
        "with min/max values: (%.6e, %.6e). \n",
        preintegrator_->get_state_jacobian().minCoeff(),
        preintegrator_->get_state_jacobian().maxCoeff());
  }

  // Bias Jacobian.
  Eigen::Matrix3d d_alpha_ba =
      preintegrator_->get_state_jacobian().block<3, 3>(0, 9);
  Eigen::Matrix3d d_alpha_bg =
      preintegrator_->get_state_jacobian().block<3, 3>(0, 12);

  Eigen::Matrix3d d_q_bg =
      preintegrator_->get_state_jacobian().block<3, 3>(3, 12);

  Eigen::Matrix3d d_beta_ba =
      preintegrator_->get_state_jacobian().block<3, 3>(6, 9);
  Eigen::Matrix3d d_beta_bg =
      preintegrator_->get_state_jacobian().block<3, 3>(6, 12);

  // Deviation from the linearization point of IMU preintegration.
  Eigen::Vector3d delta_ba = ba_i - preintegrator_->get_linearized_ba();
  Eigen::Vector3d delta_bg = bg_i - preintegrator_->get_linearized_bg();

  // Linearly update the preintegrated results.
  Eigen::Vector3d updated_alpha = preintegrator_->get_alpha_curr() +
                                  d_alpha_ba * delta_ba + d_alpha_bg * delta_bg;
  Eigen::Quaterniond updated_q =
      preintegrator_->get_q_curr() * So3ExpQuat(d_q_bg * delta_bg);
  Eigen::Vector3d updated_beta = preintegrator_->get_beta_curr() +
                                 d_beta_ba * delta_ba + d_beta_bg * delta_bg;

  double dt_sum = preintegrator_->get_dt_sum();
  Eigen::Vector3d gravity_in_W = q_WG * preintegrator_->get_gravity_in_G();

  // Compute the residual, refer to formula (6).
  residual.block<3, 1>(0, 0) =
      q_WI_i.inverse() * (p_WI_j - p_WI_i - v_WI_i * dt_sum +
                          0.5 * gravity_in_W * dt_sum * dt_sum) -
      updated_alpha;
  residual.block<3, 1>(3, 0) =
      2 * (updated_q.inverse() * (q_WI_i.inverse() * q_WI_j)).vec();
  residual.block<3, 1>(6, 0) =
      q_WI_i.inverse() * (v_WI_j - v_WI_i + gravity_in_W * dt_sum) -
      updated_beta;
  residual.block<3, 1>(9, 0) = ba_j - ba_i;
  residual.block<3, 1>(12, 0) = bg_j - bg_i;

  if (std::isnan(preintegrator_->get_meas_covariance().norm()) ||
      std::isnan(preintegrator_->get_meas_covariance().inverse().norm())) {
    PRINT_ERROR(
        "[Error]: IMU preintegration covariance exception, with invalid norm. "
        "\n");

    return false;
  }

  // Combine the infomation matrix.
  Eigen::Matrix<double, 15, 15> meas_info_sqrt =
      Eigen::LLT<Eigen::Matrix<double, 15, 15>>(
          preintegrator_->get_meas_covariance().inverse())
          .matrixL()
          .transpose();
  residual = meas_info_sqrt * residual;

  // ===========================================================================

  if (jacobians) {
    // Step 3: Calculate the Jacobian of the residual with respect to the IMU
    // pose at time i.
    if (jacobians[0]) {
      // Refer to formula (23)-(27).
      Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(
          jacobians[0]);
      jacobian_pose_i.setZero();
      // d(r_p) / d(p_i)
      jacobian_pose_i.block<3, 3>(0, 0) = -q_WI_i.inverse().toRotationMatrix();
      // d(r_p) / d(q_i)
      jacobian_pose_i.block<3, 3>(0, 3) =
          Skew(q_WI_i.inverse() * (p_WI_j - p_WI_i - v_WI_i * dt_sum +
                                   0.5 * gravity_in_W * dt_sum * dt_sum));
      // d(r_q) / d(q_i)
      jacobian_pose_i.block<3, 3>(3, 3) =
          -(QuatLeftProd(q_WI_j.inverse() * q_WI_i) * QuatRightProd(updated_q))
               .bottomRightCorner<3, 3>();
      // d(r_v) / d(q_i)
      jacobian_pose_i.block<3, 3>(6, 3) =
          Skew(q_WI_i.inverse() * (v_WI_j - v_WI_i + gravity_in_W * dt_sum));

      // Combine the infomation matrix.
      jacobian_pose_i = meas_info_sqrt * jacobian_pose_i;
    }

    // ===========================================================================

    // Step 4: Calculate the Jacobian of the residual with respect to the IMU
    // speed and bias at time i.
    if (jacobians[1]) {
      // Refer to formula (23)-(27).
      Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
          jacobian_speedbias_i(jacobians[1]);
      jacobian_speedbias_i.setZero();
      // d(r_p) / d(v_i)
      jacobian_speedbias_i.block<3, 3>(0, 0) =
          -q_WI_i.inverse().toRotationMatrix() * dt_sum;
      // d(r_p) / d(ba_i)
      jacobian_speedbias_i.block<3, 3>(0, 3) = -d_alpha_ba;
      // d(r_p) / d(bg_i)
      jacobian_speedbias_i.block<3, 3>(0, 6) = -d_alpha_bg;
      // d(r_q) / d(bg_i)
      jacobian_speedbias_i.block<3, 3>(3, 6) =
          -QuatLeftProd(q_WI_j.inverse() * q_WI_i * updated_q)
               .bottomRightCorner<3, 3>() *
          d_q_bg;
      // d(r_v) / d(v_i)
      jacobian_speedbias_i.block<3, 3>(6, 0) =
          -q_WI_i.inverse().toRotationMatrix();
      // d(r_v) / d(ba_i)
      jacobian_speedbias_i.block<3, 3>(6, 3) = -d_beta_ba;
      // d(r_v) / d(bg_i)
      jacobian_speedbias_i.block<3, 3>(6, 6) = -d_beta_bg;
      // d(r_ba) / d(ba_i)
      jacobian_speedbias_i.block<3, 3>(9, 3) = -Eigen::Matrix3d::Identity();
      // d(r_bg) / d(bg_i)
      jacobian_speedbias_i.block<3, 3>(12, 6) = -Eigen::Matrix3d::Identity();

      // Combine the infomation matrix.
      jacobian_speedbias_i = meas_info_sqrt * jacobian_speedbias_i;
    }

    // ===========================================================================

    // Step 5: Calculate the Jacobian of the residual with respect to the IMU
    // pose at time j.
    if (jacobians[2]) {
      // Refer to formula (23)-(27).
      Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(
          jacobians[2]);
      jacobian_pose_j.setZero();
      // d(r_p) / d(p_j)
      jacobian_pose_j.block<3, 3>(0, 0) = q_WI_i.inverse().toRotationMatrix();
      // d(r_q) / d(q_j)
      jacobian_pose_j.block<3, 3>(3, 3) =
          QuatLeftProd(updated_q.inverse() * q_WI_i.inverse() * q_WI_j)
              .bottomRightCorner<3, 3>();

      // Combine the infomation matrix.
      jacobian_pose_j = meas_info_sqrt * jacobian_pose_j;
    }

    // ===========================================================================

    // Step 6: Calculate the Jacobian of the residual with respect to the IMU
    // speed bias at time j.
    if (jacobians[3]) {
      // Refer to formula (23)-(27).
      Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
          jacobian_speedbias_j(jacobians[3]);
      jacobian_speedbias_j.setZero();
      // d(r_v) / d(v_j)
      jacobian_speedbias_j.block<3, 3>(6, 0) =
          q_WI_i.inverse().toRotationMatrix();
      // d(r_ba) / d(ba_j)
      jacobian_speedbias_j.block<3, 3>(9, 3) = Eigen::Matrix3d::Identity();
      // d(r_bg) / d(bg_j)
      jacobian_speedbias_j.block<3, 3>(12, 6) = Eigen::Matrix3d::Identity();

      // Combine the infomation matrix.
      jacobian_speedbias_j = meas_info_sqrt * jacobian_speedbias_j;
    }

    // ===========================================================================

    // Step 7: Calculate the Jacobian of the residual with respect to the
    // 2-DoF(roll pitch) gravity aligned rotation.
    if (jacobians[4]) {
      // Refer to formula (28)-(32).
      Eigen::Map<Eigen::Matrix<double, 15, 2, Eigen::RowMajor>> jacobian_rotxy(
          jacobians[4]);
      jacobian_rotxy.setZero();
      // d(g_w) / d(rot_xy)
      Eigen::Matrix<double, 3, 2> d_gw_rotxy =
          Eigen::Matrix<double, 3, 2>::Zero();
      d_gw_rotxy << -std::sin(pitch) * std::sin(roll),
          std::cos(pitch) * std::cos(roll), -std::cos(roll), 0.,
          -std::cos(pitch) * std::sin(roll), -std::sin(pitch) * std::cos(roll);
      // d(r_p) / d(rot_xy)
      jacobian_rotxy.block<3, 2>(0, 0) =
          0.5 * q_WI_i.toRotationMatrix().inverse() * dt_sum * dt_sum *
          gravity_in_W.norm() * d_gw_rotxy;
      // d(r_v) / d(rot_xy)
      jacobian_rotxy.block<3, 2>(6, 0) = q_WI_i.toRotationMatrix().inverse() *
                                         dt_sum * gravity_in_W.norm() *
                                         d_gw_rotxy;

      // Combine the infomation matrix.
      jacobian_rotxy = meas_info_sqrt * jacobian_rotxy;
    }

    // ===========================================================================
  }

  return true;
}

}  // namespace mocap2gt
