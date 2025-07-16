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

#include "mocap2gt/ceres_factor/mocap_interpolation_factor.h"

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
  }
  for (int i = 0; i < 4; ++i) {
    para_curr[0][i + 3] = para_backup[0][i + 3];
    para_curr[1][i + 3] = para_backup[1][i + 3];
  }
  para_curr[2][0] = para_backup[2][0];
}

int main() {
  mocap2gt::Printer::SetPrintLevel(mocap2gt::Printer::PrintLevel::ALL);

  const Eigen::Vector3d kTransNoise(0.0005, 0.0005, 0.0005);
  const Eigen::Vector3d kRotNoise(0.001, 0.001, 0.001);
  const bool kUseSplineFlag = false;

  // Step 1: Set up the Mocap data and initialize the preintegrator factor.
  Eigen::Matrix<double, 20, 8> mocap_data_example =
      Eigen::Matrix<double, 20, 8>::Zero();
  // clang-format off
  mocap_data_example <<
    0.01, 1.986041622, 2.112284553, 1.554899703,
      -0.008085549, 0.012630617, -0.726848080, 0.686634517,
    0.02, 1.986083668, 2.108729074, 1.555857886,
      -0.007571676, 0.011300773, -0.727016519, 0.686485209,
    0.03, 1.985553320, 2.105891456, 1.557357845,
      -0.003723774, 0.007408663, -0.727040960, 0.686544018,
    0.04, 1.986210426, 2.101877324, 1.558464038,
      -0.007509633, 0.009987868, -0.727070291, 0.686449299,
    0.05, 1.986942815, 2.098929260, 1.559661787,
      -0.008227769, 0.011082452, -0.728212538, 0.685212363,
    0.06, 1.987260774, 2.095352900, 1.561078250,
      -0.008631574, 0.010486822, -0.727737900, 0.685720841,
    0.07, 1.987444549, 2.091936757, 1.562248072,
      -0.008626971, 0.010036321, -0.728051526, 0.685394648,
    0.08, 1.987500992, 2.088684945, 1.563541289,
      -0.007248173, 0.008572565, -0.728120332, 0.685357394,
    0.09, 1.987650227, 2.085164570, 1.564441506,
      -0.008066713, 0.008096368, -0.728178046, 0.685292719,
    0.10, 1.987941004, 2.081863938, 1.565756780,
      -0.006356923, 0.007574901, -0.728849221, 0.684602822,
    0.11, 1.988927717, 2.078509571, 1.567748140,
      -0.008868532, 0.009703255, -0.728285013, 0.685148259,
    0.12, 1.988918834, 2.075379741, 1.568506610,
      -0.008115326, 0.007497690, -0.728603827, 0.684846252,
    0.13, 1.989308603, 2.071935132, 1.570165685,
      -0.006654465, 0.006835202, -0.728839375, 0.684618261,
    0.14, 1.988610169, 2.069452325, 1.571280292,
      -0.002922751, -0.00020816, -0.728408399, 0.685136934,
    0.15, 1.989440890, 2.065452349, 1.572794957,
      -0.004978477, 0.004228272, -0.728804319, 0.684690880,
    0.16, 1.989834459, 2.062100835, 1.574402974,
      -0.003582087, 0.003544737, -0.729626458, 0.683827343,
    0.17, 1.990316164, 2.058770534, 1.575644970,
      -0.003999142, 0.003702722, -0.729618531, 0.683832652,
    0.18, 1.990084091, 2.055735991, 1.576739716,
      -0.002969053, 0.001073259, -0.729891833, 0.683555370,
    0.19, 1.990956182, 2.052869062, 1.578749963,
      -0.001896092, 0.000493494, -0.730161364, 0.683271940,
    0.20, 1.990931877, 2.049172574, 1.579488283,
      -0.001114594, 0.000986549, -0.730457247, 0.682956803;
  // clang-format on

  // Initialized the MoCap measurement manager.
  std::shared_ptr<mocap2gt::MoCapMeasManagerBase> manager;
  if (kUseSplineFlag) {
    PRINT_DEBUG(
        "[Debug]: Use the cubic B-spline interpolation model of MoCap data "
        "manager. "
        "\n");

    manager.reset(new mocap2gt::MoCapMeasManagerSpline(kTransNoise, kRotNoise));
  } else {
    PRINT_DEBUG(
        "[Debug]: Use the linear interpolation model of MoCap data manager. "
        "\n");

    manager.reset(new mocap2gt::MoCapMeasManagerLinear(kTransNoise, kRotNoise));
  }

  for (int i = 0; i < mocap_data_example.rows(); ++i) {
    double timestamp = mocap_data_example.row(i)(0);
    Eigen::Vector3d p = mocap_data_example.block<1, 3>(i, 1);
    Eigen::Quaterniond q = Eigen::Quaterniond(
        mocap_data_example.row(i)(7), mocap_data_example.row(i)(4),
        mocap_data_example.row(i)(5), mocap_data_example.row(i)(6));
    manager->FeedPose(timestamp, p, q);
  }

  if (kUseSplineFlag) {
    manager->GenerateControlPoints();
  }

  // Initialize the MoCap interpolation factor.
  double state_timestamp = mocap_data_example.row(4)(0) + 0.002;
  std::shared_ptr<mocap2gt::MoCapInterpolationFactor> factor(
      new mocap2gt::MoCapInterpolationFactor(manager, state_timestamp));

  // ===========================================================================

  // Step 2: Allocate the memory, and set up the involved states.
  // Involved sates.
  double **param = new double *[3];
  // States backup.
  double **param_backup = new double *[3];
  // 6-dimensional preintegration residual.
  double *residual = new double[6];
  // Jacobians of residual with respect to each state.
  double **jacobian = new double *[3];

  param[0] = new double[7];
  param[1] = new double[7];
  param[2] = new double[1];

  param_backup[0] = new double[7];
  param_backup[1] = new double[7];
  param_backup[2] = new double[1];

  jacobian[0] = new double[6 * 7];
  jacobian[1] = new double[6 * 7];
  jacobian[2] = new double[6 * 1];

  // Set up random states.
  Eigen::Vector3d p_WI_i = Eigen::Vector3d::Random();
  Eigen::Quaterniond q_WI_i = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector3d p_MI = Eigen::Vector3d::Random();
  Eigen::Quaterniond q_MI = Eigen::Quaterniond::UnitRandom();
  double toff_MI = (Eigen::VectorXd::Random(1))[0] * 0.001;

  // Backup.
  for (int i = 0; i < 3; ++i) {
    param_backup[0][i] = p_WI_i[i];
    param_backup[1][i] = p_MI[i];
  }
  for (int i = 0; i < 4; ++i) {
    param_backup[0][i + 3] = q_WI_i.coeffs()[i];
    param_backup[1][i + 3] = q_MI.coeffs()[i];
  }
  param_backup[2][0] = toff_MI;

  resetParam(param, param_backup);

  // ===========================================================================

  // Step 3: Calculate initial residual and Jacobian based on initial states.
  factor->Evaluate(param, residual, jacobian);

  Eigen::Matrix<double, 6, 6> meas_cov = Eigen::Matrix<double, 6, 6>::Zero();
  meas_cov.block<3, 3>(0, 0) =
      kTransNoise.array().square().matrix().asDiagonal();
  meas_cov.block<3, 3>(3, 3) = kRotNoise.array().square().matrix().asDiagonal();
  Eigen::Matrix<double, 6, 6> meas_info_sqrt =
      Eigen::LLT<Eigen::Matrix<double, 6, 6>>(meas_cov.inverse())
          .matrixL()
          .transpose();

  // Residual and Jacobian combined with information matrix.
  Eigen::Map<Eigen::Matrix<double, 6, 1>> residual_info(residual);
  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jac_pose_i_info(
      jacobian[0]);
  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jac_extrinsic_info(
      jacobian[1]);
  Eigen::Map<Eigen::Matrix<double, 6, 1>> jac_toff_info(jacobian[2]);

  // Recover the original residual and Jacobian.
  Eigen::Matrix<double, 6, 1> residual_orig =
      meas_info_sqrt.inverse() * residual_info;
  Eigen::Matrix<double, 6, 7> jac_pose_i_orig =
      meas_info_sqrt.inverse() * jac_pose_i_info;
  Eigen::Matrix<double, 6, 7> jac_extrinsic_orig =
      meas_info_sqrt.inverse() * jac_extrinsic_info;
  Eigen::Matrix<double, 6, 1> jac_toff_orig =
      meas_info_sqrt.inverse() * jac_toff_info;

  // ===========================================================================

  // Step 4: Add perturbations to each state and check the residual and the
  // Jacobian.
  const Eigen::Vector3d kPerturbVec = Eigen::Vector3d::Random() * 0.001;
  const double kPerturbToff = (Eigen::VectorXd::Random(1))[0] * 0.001;
  PRINT_DEBUG(
      "[Debug]: Bellow, we will check the residual and Jacobian of the "
      "preintegration factor. \n");
  PRINT_DEBUG(
      "[Debug]: If no exception, after perturbation, the \"residual diff\". "
      "and "
      "the \"Jacobian diff\" should be close. \n");

  // Step 4.1: Perturb the IMU pose at time i.
  PRINT_DEBUG("================ Perturb pose at time i. ===============\n");
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
  Eigen::Map<Eigen::Matrix<double, 6, 1>> residual_perturb_info(residual);
  Eigen::Matrix<double, 6, 1> residual_perturb_orig =
      meas_info_sqrt.inverse() * residual_perturb_info;
  std::cout << std::setprecision(4);
  std::cout << "Residual diff: "
            << (residual_perturb_orig - residual_orig).transpose() << std::endl;
  std::cout << "Jacobian diff: "
            << (jac_pose_i_orig * perturb_pose_i).transpose() << std::endl;

  // ===========================================================================

  // Step 4.2: Perturb the extrinsic.
  PRINT_DEBUG("================== Perturb extrinsic. ==================\n");
  // Reset and perturb.
  p_MI += kPerturbVec;
  q_MI *= mocap2gt::So3ExpQuat(kPerturbVec);
  Eigen::Matrix<double, 7, 1> perturb_extrinsic =
      Eigen::Matrix<double, 7, 1>::Zero();
  perturb_extrinsic.block<3, 1>(0, 0) = kPerturbVec;
  perturb_extrinsic.block<3, 1>(3, 0) = kPerturbVec;
  resetParam(param, param_backup);
  for (int i = 0; i < 3; ++i) {
    param[1][i] = p_MI[i];
  }
  for (int i = 0; i < 4; ++i) {
    param[1][i + 3] = q_MI.coeffs()[i];
  }

  // Compare the "residual diff" and "Jacobian diff".
  factor->Evaluate(param, residual, jacobian);
  residual_perturb_info = Eigen::Map<Eigen::Matrix<double, 6, 1>>(residual);
  residual_perturb_orig = meas_info_sqrt.inverse() * residual_perturb_info;
  std::cout << "Residual diff: "
            << (residual_perturb_orig - residual_orig).transpose() << std::endl;
  std::cout << "Jacobian diff: "
            << (jac_extrinsic_orig * perturb_extrinsic).transpose()
            << std::endl;

  // ===========================================================================

  // Step 4.3: Perturb the time offset.
  PRINT_DEBUG("================= Perturb time offset. =================\n");
  // Reset and perturb.
  toff_MI += kPerturbToff;
  resetParam(param, param_backup);
  param[2][0] = toff_MI;

  // Compare the "residual diff" and "Jacobian diff".
  factor->Evaluate(param, residual, jacobian);
  residual_perturb_info = Eigen::Map<Eigen::Matrix<double, 6, 1>>(residual);
  residual_perturb_orig = meas_info_sqrt.inverse() * residual_perturb_info;
  std::cout << "Residual diff: "
            << (residual_perturb_orig - residual_orig).transpose() << std::endl;
  std::cout << "Jacobian diff: " << (jac_toff_orig * kPerturbToff).transpose()
            << std::endl;

  // ===========================================================================

  // Step 5: Free memory.
  for (int i = 0; i < 3; ++i) {
    delete[] param[i];
  }
  for (int i = 0; i < 3; ++i) {
    delete[] jacobian[i];
  }
  delete[] param;
  delete[] jacobian;
  delete[] residual;

  // ===========================================================================

  return 0;
}
