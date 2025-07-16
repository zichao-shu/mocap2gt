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

#include <iomanip>
#include <iostream>

#include "mocap2gt/ceres_factor/imu_preintegration_factor.h"
#include "mocap2gt/sensor_meas/imu_preintegrator.h"

/**
 * @brief Rollback the states to the backup.
 *
 * @param para_curr Current state.
 * @param para_backup State backup.
 */
void resetParam(double **para_curr, double **para_backup) {
  for (int i = 0; i < 3; ++i) {
    para_curr[0][i] = para_backup[0][i];
    para_curr[1][i] = para_backup[1][i];
    para_curr[1][i + 3] = para_backup[1][i + 3];
    para_curr[1][i + 6] = para_backup[1][i + 6];
    para_curr[2][i] = para_backup[2][i];
    para_curr[3][i] = para_backup[3][i];
    para_curr[3][i + 3] = para_backup[3][i + 3];
    para_curr[3][i + 6] = para_backup[3][i + 6];
  }
  for (int i = 0; i < 4; ++i) {
    para_curr[0][i + 3] = para_backup[0][i + 3];
    para_curr[2][i + 3] = para_backup[2][i + 3];
  }
  for (int i = 0; i < 2; ++i) {
    para_curr[4][i] = para_backup[4][i];
  }
}

int main() {
  mocap2gt::Printer::SetPrintLevel(mocap2gt::Printer::PrintLevel::ALL);

  const Eigen::Vector3d kBa(0.01, 0.01, 0.01);
  const Eigen::Vector3d kBg(0.001, 0.001, 0.001);
  const Eigen::Vector4d kImuNoise(2e-3, 3e-3, 2e-4, 2e-5);
  constexpr double kGravityMagnitude = 9.81;

  // Step 1: Set up the IMU data and initialize the preintegrator factor.
  Eigen::Matrix<double, 4, 7> imu_data_example =
      Eigen::Matrix<double, 4, 7>::Zero();
  // clang-format off
  imu_data_example <<
    1034777.907, -0.00247252, 0.00282198, -0.00412735,
      -0.0458349, -9.78181, -0.133548,
    1034777.917, -0.00247252, 0.00282198, -0.00412735,
      -0.0482291, -9.77462, -0.135943,
    1034777.927, -0.00247252, 0.00388727, -0.00412735,
      -0.0434407, -9.78419, -0.138337,
    1034777.937, -0.00247252, 0.00388727, -0.00306205,
      -0.0458349, -9.77701, -0.128761;
  // clang-format on

  // Initialize the IMU preintegrator and perform the propagation.
  std::shared_ptr<mocap2gt::ImuPreintegrator> preintegrator(
      new mocap2gt::ImuPreintegrator(
          imu_data_example.block<1, 3>(0, 4),
          imu_data_example.block<1, 3>(0, 1), kBa, kBg, kImuNoise[0],
          kImuNoise[1], kImuNoise[2], kImuNoise[3], kGravityMagnitude));

  for (int i = 1; i < imu_data_example.rows(); ++i) {
    double dt = imu_data_example(i, 0) - imu_data_example(i - 1, 0);
    preintegrator->Propagate(dt, imu_data_example.block<1, 3>(i, 4),
                             imu_data_example.block<1, 3>(i, 1));
  }

  // Initialize the IMU preintegrator factor.
  std::shared_ptr<mocap2gt::IMUPreintegrationFactor> factor(
      new mocap2gt::IMUPreintegrationFactor(preintegrator));

  // ===========================================================================

  // Step 2: Allocate the memory, and set up the involved states.
  // Involved sates.
  double **param = new double *[5];
  // States backup.
  double **param_backup = new double *[5];
  // 15-dimensional preintegration residual.
  double *residual = new double[15];
  // Jacobians of residual with respect to each state.
  double **jacobian = new double *[5];

  param[0] = new double[7];
  param[1] = new double[9];
  param[2] = new double[7];
  param[3] = new double[9];
  param[4] = new double[2];

  param_backup[0] = new double[7];
  param_backup[1] = new double[9];
  param_backup[2] = new double[7];
  param_backup[3] = new double[9];
  param_backup[4] = new double[2];

  jacobian[0] = new double[15 * 7];
  jacobian[1] = new double[15 * 9];
  jacobian[2] = new double[15 * 7];
  jacobian[3] = new double[15 * 9];
  jacobian[4] = new double[15 * 2];

  // Set up random states.
  Eigen::Vector3d p_WI_i = Eigen::Vector3d::Random();
  Eigen::Quaterniond q_WI_i = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector3d v_WI_i = Eigen::Vector3d::Random();
  Eigen::Vector3d ba_i = Eigen::Vector3d::Random() * 0.01;
  Eigen::Vector3d bg_i = Eigen::Vector3d::Random() * 0.001;
  Eigen::Vector3d p_WI_j = Eigen::Vector3d::Random();
  Eigen::Quaterniond q_WI_j = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector3d v_WI_j = Eigen::Vector3d::Random();
  Eigen::Vector3d ba_j = ba_i + Eigen::Vector3d::Random() * 0.0001;
  Eigen::Vector3d bg_j = bg_i + Eigen::Vector3d::Random() * 0.00001;
  Eigen::Vector2d rotxy_WG = Eigen::Vector2d::Zero();

  // Backup.
  for (int i = 0; i < 3; ++i) {
    param_backup[0][i] = p_WI_i[i];
    param_backup[1][i] = v_WI_i[i];
    param_backup[1][i + 3] = ba_i[i];
    param_backup[1][i + 6] = bg_i[i];
    param_backup[2][i] = p_WI_j[i];
    param_backup[3][i] = v_WI_j[i];
    param_backup[3][i + 3] = ba_j[i];
    param_backup[3][i + 6] = bg_j[i];
  }
  for (int i = 0; i < 4; ++i) {
    param_backup[0][i + 3] = q_WI_i.coeffs()[i];
    param_backup[2][i + 3] = q_WI_j.coeffs()[i];
  }
  for (int i = 0; i < 2; ++i) {
    param_backup[4][i] = rotxy_WG[i];
  }

  resetParam(param, param_backup);

  // ===========================================================================

  // Step 3: Calculate initial residual and Jacobian based on initial states.
  factor->Evaluate(param, residual, jacobian);

  Eigen::Matrix<double, 15, 15> meas_info_sqrt =
      Eigen::LLT<Eigen::Matrix<double, 15, 15>>(
          preintegrator->get_meas_covariance().inverse())
          .matrixL()
          .transpose();

  // Residual and Jacobian combined with information matrix.
  Eigen::Map<Eigen::Matrix<double, 15, 1>> residual_info(residual);
  Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jac_pose_i_info(
      jacobian[0]);
  Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
      jac_speedbias_i_info(jacobian[1]);
  Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jac_pose_j_info(
      jacobian[2]);
  Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
      jac_speedbias_j_info(jacobian[3]);
  Eigen::Map<Eigen::Matrix<double, 15, 2, Eigen::RowMajor>> jac_rotxy_info(
      jacobian[4]);

  // Recover the original residual and Jacobian.
  Eigen::Matrix<double, 15, 1> residual_orig =
      meas_info_sqrt.inverse() * residual_info;
  Eigen::Matrix<double, 15, 7> jac_pose_i_orig =
      meas_info_sqrt.inverse() * jac_pose_i_info;
  Eigen::Matrix<double, 15, 9> jac_speedbias_i_orig =
      meas_info_sqrt.inverse() * jac_speedbias_i_info;
  Eigen::Matrix<double, 15, 7> jac_pose_j_orig =
      meas_info_sqrt.inverse() * jac_pose_j_info;
  Eigen::Matrix<double, 15, 9> jac_speedbias_j_orig =
      meas_info_sqrt.inverse() * jac_speedbias_j_info;
  Eigen::Matrix<double, 15, 2> jac_rotxy_orig =
      meas_info_sqrt.inverse() * jac_rotxy_info;

  // ===========================================================================

  // Step 4: Add perturbations to each state and check the residual and the
  // Jacobian.
  const Eigen::Vector3d kPerturbVec = Eigen::Vector3d::Random() * 0.001;
  Eigen::Vector2d kPerturbRotXY = Eigen::Vector2d::Random() * 0.001;
  PRINT_DEBUG(
      "[Debug]: Bellow, we will check the residual and Jacobian of the "
      "preintegration factor. \n");
  PRINT_DEBUG(
      "[Debug]: If no exception, after perturbation, the \"residual diff\". and "
      "the \"Jacobian diff\" should be close. \n");

  // Step 4.1: Perturb the IMU pose at time i.
  PRINT_DEBUG(
      "===================================== Perturb pose at time i. "
      "=====================================\n");
  // Reset and perturb.
  p_WI_i += kPerturbVec;
  q_WI_i *= mocap2gt::So3ExpQuat(kPerturbVec);
  Eigen::Matrix<double, 7, 1> perturb_pose_i =
      Eigen::Matrix<double, 7, 1>::Zero();
  perturb_pose_i.block<3, 1>(0, 0) = kPerturbVec;
  perturb_pose_i.block<3, 1>(3, 0) = kPerturbVec;
  resetParam(param, param_backup);
  for (int i = 0; i < 3; ++i) {
    param[0][i] = p_WI_i[i];
  }
  for (int i = 0; i < 4; ++i) {
    param[0][i + 3] = q_WI_i.coeffs()[i];
  }

  // Compare the "residual diff" and "Jacobian diff".
  factor->Evaluate(param, residual, jacobian);
  Eigen::Map<Eigen::Matrix<double, 15, 1>> residual_perturb_info(residual);
  Eigen::Matrix<double, 15, 1> residual_perturb_orig =
      meas_info_sqrt.inverse() * residual_perturb_info;
  std::cout << std::setprecision(4);
  std::cout << "Residual diff: "
            << (residual_perturb_orig - residual_orig).transpose() << std::endl;
  std::cout << "Jacobian diff: "
            << (jac_pose_i_orig * perturb_pose_i).transpose() << std::endl;

  // ===========================================================================

  // Step 4.2: Perturb the IMU speed bias at time i.
  PRINT_DEBUG(
      "================================ Perturb speed and bias at time i. "
      "================================\n");
  // Reset and perturb.
  v_WI_i += kPerturbVec;
  ba_i += kPerturbVec;
  bg_i += kPerturbVec;
  Eigen::Matrix<double, 9, 1> perturb_speedbias_i =
      Eigen::Matrix<double, 9, 1>::Zero();
  perturb_speedbias_i.block<3, 1>(0, 0) = kPerturbVec;
  perturb_speedbias_i.block<3, 1>(3, 0) = kPerturbVec;
  perturb_speedbias_i.block<3, 1>(6, 0) = kPerturbVec;
  resetParam(param, param_backup);
  for (int i = 0; i < 3; ++i) {
    param[1][i] = v_WI_i[i];
    param[1][i + 3] = ba_i[i];
    param[1][i + 6] = bg_i[i];
  }

  // Compare the "residual diff" and "Jacobian diff".
  factor->Evaluate(param, residual, jacobian);
  residual_perturb_info = Eigen::Map<Eigen::Matrix<double, 15, 1>>(residual);
  residual_perturb_orig = meas_info_sqrt.inverse() * residual_perturb_info;
  std::cout << "Residual diff: "
            << (residual_perturb_orig - residual_orig).transpose() << std::endl;
  std::cout << "Jacobian diff: "
            << (jac_speedbias_i_orig * perturb_speedbias_i).transpose()
            << std::endl;

  // ===========================================================================

  // Step 4.3: Perturb the IMU pose at time j.
  PRINT_DEBUG(
      "===================================== Perturb pose at time j. "
      "=====================================\n");
  // Reset and perturb.
  p_WI_j += kPerturbVec;
  q_WI_j *= mocap2gt::So3ExpQuat(kPerturbVec);
  Eigen::Matrix<double, 7, 1> perturb_pose_j =
      Eigen::Matrix<double, 7, 1>::Zero();
  perturb_pose_j.block<3, 1>(0, 0) = kPerturbVec;
  perturb_pose_j.block<3, 1>(3, 0) = kPerturbVec;
  resetParam(param, param_backup);
  for (int i = 0; i < 3; ++i) {
    param[2][i] = p_WI_j[i];
  }
  for (int i = 0; i < 4; ++i) {
    param[2][i + 3] = q_WI_j.coeffs()[i];
  }

  // Compare the "residual diff" and "Jacobian diff".
  factor->Evaluate(param, residual, jacobian);
  residual_perturb_info = Eigen::Map<Eigen::Matrix<double, 15, 1>>(residual);
  residual_perturb_orig = meas_info_sqrt.inverse() * residual_perturb_info;
  std::cout << "Residual diff: "
            << (residual_perturb_orig - residual_orig).transpose() << std::endl;
  std::cout << "Jacobian diff: "
            << (jac_pose_j_orig * perturb_pose_j).transpose() << std::endl;

  // ===========================================================================

  // Step 4.4: Perturb the IMU speed bias at time j.
  PRINT_DEBUG(
      "================================ Perturb speed and bias at time j. "
      "================================\n");
  // Reset and perturb.
  v_WI_j += kPerturbVec;
  ba_j += kPerturbVec;
  bg_j += kPerturbVec;
  Eigen::Matrix<double, 9, 1> perturb_speedbias_j =
      Eigen::Matrix<double, 9, 1>::Zero();
  perturb_speedbias_j.block<3, 1>(0, 0) = kPerturbVec;
  perturb_speedbias_j.block<3, 1>(3, 0) = kPerturbVec;
  perturb_speedbias_j.block<3, 1>(6, 0) = kPerturbVec;
  resetParam(param, param_backup);
  for (int i = 0; i < 3; ++i) {
    param[3][i] = v_WI_j[i];
    param[3][i + 3] = ba_j[i];
    param[3][i + 6] = bg_j[i];
  }

  // Compare the "residual diff" and "Jacobian diff".
  factor->Evaluate(param, residual, jacobian);
  residual_perturb_info = Eigen::Map<Eigen::Matrix<double, 15, 1>>(residual);
  residual_perturb_orig = meas_info_sqrt.inverse() * residual_perturb_info;
  std::cout << "Residual diff: "
            << (residual_perturb_orig - residual_orig).transpose() << std::endl;
  std::cout << "Jacobian diff: "
            << (jac_speedbias_j_orig * perturb_speedbias_j).transpose()
            << std::endl;

  // ===========================================================================

  // Step 4.5: Perturb the gravity aligned rotation.
  PRINT_DEBUG(
      "================================ Perturb gravity aligned rotation. "
      "=================================\n");
  // Reset and perturb.
  rotxy_WG += kPerturbRotXY;
  resetParam(param, param_backup);
  for (int i = 0; i < 2; ++i) {
    param[4][i] = rotxy_WG[i];
  }

  // Compare the "residual diff" and "Jacobian diff".
  factor->Evaluate(param, residual, jacobian);
  residual_perturb_info = Eigen::Map<Eigen::Matrix<double, 15, 1>>(residual);
  residual_perturb_orig = meas_info_sqrt.inverse() * residual_perturb_info;
  std::cout << "Residual diff: "
            << (residual_perturb_orig - residual_orig).transpose() << std::endl;
  std::cout << "Jacobian diff: " << (jac_rotxy_orig * kPerturbRotXY).transpose()
            << std::endl;

  // ===========================================================================

  // Step 5: Free memory.
  for (int i = 0; i < 5; ++i) {
    delete[] param[i];
  }
  for (int i = 0; i < 5; ++i) {
    delete[] jacobian[i];
  }
  delete[] param;
  delete[] jacobian;
  delete[] residual;

  // ===========================================================================

  return 0;
}
