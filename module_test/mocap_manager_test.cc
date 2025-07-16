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
#include <memory>

#include "mocap2gt/sensor_meas/mocap_meas_manager_linear.h"
#include "mocap2gt/sensor_meas/mocap_meas_manager_spline.h"

int main() {
  mocap2gt::Printer::SetPrintLevel(mocap2gt::Printer::PrintLevel::ALL);

  const Eigen::Vector3d kTransNoise(0.0005, 0.0005, 0.0005);
  const Eigen::Vector3d kRotNoise(0.001, 0.001, 0.001);
  const bool kUseSplineFlag = true;

  // Set up the test data.
  Eigen::Matrix<double, 20, 8> mocap_data_example;
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
        "[Debug]: Use the linear interpolation model of MoCap data manager. "
        "\n");

    manager.reset(new mocap2gt::MoCapMeasManagerSpline(kTransNoise, kRotNoise));
  } else {
    PRINT_DEBUG(
        "[Debug]: Use the cubic B-spline interpolation model of MoCap data "
        "manager. "
        "\n");

    manager.reset(new mocap2gt::MoCapMeasManagerLinear(kTransNoise, kRotNoise));
  }

  PRINT_DEBUG(
      "================== Feed MoCap poses for test. ==================\n");
  for (int i = 0; i < mocap_data_example.rows(); ++i) {
    double timestamp = mocap_data_example.row(i)(0);
    Eigen::Vector3d p = mocap_data_example.block<1, 3>(i, 1);
    Eigen::Quaterniond q = Eigen::Quaterniond(
        mocap_data_example.row(i)(7), mocap_data_example.row(i)(4),
        mocap_data_example.row(i)(5), mocap_data_example.row(i)(6));
    manager->FeedPose(timestamp, p, q);
  }

  PRINT_DEBUG(
      "[Debug]: Feed a total of %d data, with timestamps ranging from %f to "
      "%f. \n",
      manager->get_data_count(), manager->get_start_time(),
      manager->get_end_time());

  if (kUseSplineFlag) {
    manager->GenerateControlPoints();
  }

  // ===========================================================================

  PRINT_DEBUG(
      "====================== Test MoCap manager. =====================\n");
  // Test if the manager can detect interpolation time exceeding time range.
  double target_timestamp = 0.020 + 1e-6;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  Eigen::Matrix<double, 6, 6> cov;
  Eigen::Matrix<double, 6, 1> J_pose_toff;
  manager->GetPoseWithJacobian(target_timestamp, &p, &q, &cov, &J_pose_toff);

  // Check the Jacobian of output pose with respect to time offset.
  PRINT_DEBUG(
      "[Debug]: Below, we will check the the Jacobian of output pose with "
      "respect to time offset. \n");
  PRINT_DEBUG(
      "[Debug]: If no exception， the \"interpolation diff\" and the "
      "\"Jacobian "
      "diff\" should be close. \n");
  target_timestamp = 0.1745;
  Eigen::Vector3d p_0;
  Eigen::Quaterniond q_0;
  Eigen::Matrix<double, 6, 6> cov_0;
  Eigen::Matrix<double, 6, 1> J_pose_toff_0;
  manager->GetPoseWithJacobian(target_timestamp, &p_0, &q_0, &cov_0,
                               &J_pose_toff_0);

  // Add the perturbation.
  constexpr double kPerturb = 0.0005;
  PRINT_DEBUG("[Debug]: Add perturbation time offset: %f. \n", kPerturb);
  target_timestamp -= kPerturb;
  Eigen::Vector3d p_1;
  Eigen::Quaterniond q_1;
  Eigen::Matrix<double, 6, 6> cov_1;
  Eigen::Matrix<double, 6, 1> J_pose_toff_1;
  manager->GetPoseWithJacobian(target_timestamp, &p_1, &q_1, &cov_1,
                               &J_pose_toff_1);

  // Calculate the difference.
  Eigen::Vector3d p_interp_diff = p_1 - p_0;
  Eigen::Quaterniond q_interp_diff = q_0.inverse() * q_1;
  Eigen::Vector3d p_jacob_diff = J_pose_toff_0.block<3, 1>(0, 0) * kPerturb;
  Eigen::Vector3d r_jacob_diff = J_pose_toff_0.block<3, 1>(3, 0) * kPerturb;
  Eigen::Quaterniond q_jacob_diff(mocap2gt::So3Exp(r_jacob_diff));
  PRINT_DEBUG("[Debug]: Trans interpolation diff: (%.9f, %.9f, %.9f). \n",
              p_interp_diff.x(), p_interp_diff.y(), p_interp_diff.z());
  PRINT_DEBUG("[Debug]: Trans Jacobian diff     : (%.9f, %.9f, %.9f). \n",
              p_jacob_diff.x(), p_jacob_diff.y(), p_jacob_diff.z());
  PRINT_DEBUG("[Debug]: Rot interpolation diff: (%.9f, %.9f, %.9f, %.9f). \n",
              q_interp_diff.x(), q_interp_diff.y(), q_interp_diff.z(),
              q_interp_diff.w());
  PRINT_DEBUG("[Debug]: Rot Jacobian diff     : (%.9f, %.9f, %.9f, %.9f). \n",
              q_jacob_diff.x(), q_jacob_diff.y(), q_jacob_diff.z(),
              q_jacob_diff.w());

  // ===========================================================================

  return 0;
}
