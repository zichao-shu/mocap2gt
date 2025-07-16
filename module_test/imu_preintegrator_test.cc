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

#include "mocap2gt/sensor_meas/imu_preintegrator.h"

#include <iomanip>
#include <iostream>
#include <memory>

#include "mocap2gt/utils/printer.h"

/**
 * @brief Define the friend function to test IMU preintegration.
 *
 * @param integrator_ptr Preintegrator under test.
 * @param dt Time interval for a single propagation in the test.
 * @param acc Accelerometer measurement for a single propagation in the test.
 * @param gyr Gyroscope measurement for a single propagation in the test.
 *
 * We add perturbation in different states to cross-check the preintegration
 * propagation and Jacobian calculation by comparing the propagated result with
 * the linearized Jacobian result.
 */
void mocap2gt::CheckJacobian(mocap2gt::ImuPreintegrator *integrator_ptr,
                             const double &dt, const Eigen::Vector3d &acc,
                             const Eigen::Vector3d &gyr) {
  // Step 1: Initialize setting.
  // IMU data backup.
  Eigen::Vector3d acc_0_backup = integrator_ptr->acc_m_0_;
  Eigen::Vector3d gyr_0_backup = integrator_ptr->gyr_m_0_;
  Eigen::Vector3d alpha_backup = integrator_ptr->alpha_last_;
  Eigen::Quaterniond q_backup = integrator_ptr->q_last_;
  Eigen::Vector3d beta_backup = integrator_ptr->beta_last_;
  Eigen::Vector3d ba_backup = integrator_ptr->linearized_ba_;
  Eigen::Vector3d bg_backup = integrator_ptr->linearized_bg_;

  // Perform a single propagation and obtain the integration result.
  integrator_ptr->Propagate(dt, acc, gyr);
  Eigen::Vector3d alpha_propagate = integrator_ptr->alpha_curr_;
  Eigen::Quaterniond q_propagate = integrator_ptr->q_curr_;
  Eigen::Vector3d beta_propagate = integrator_ptr->beta_curr_;

  // Set the perturbation vector.
  const Eigen::Vector3d kPerturbVec(1e-4, 4e-4, 8e-4);
  PRINT_DEBUG(
      "[Debug]: Below, we will check the preintegration propagation and the "
      "Jacobian calculation. \n");
  PRINT_DEBUG(
      "[Debug]: If no exception, after perturbation, the \"propagation diff\" "
      "and the \"Jacobian diff\" should be close. \n");
  PRINT_DEBUG("[Debug]: Add perturbation vector: (%.6f, %.6f, %.6f). \n",
              kPerturbVec.x(), kPerturbVec.y(), kPerturbVec.z());

  // ===========================================================================

  // Step 2: Perturb alpha and check the result.
  PRINT_DEBUG("============= Perturb alpha. ==============\n");
  integrator_ptr->acc_m_0_ = acc_0_backup;
  integrator_ptr->gyr_m_0_ = gyr_0_backup;
  integrator_ptr->alpha_last_ = alpha_backup + kPerturbVec;
  integrator_ptr->q_last_ = q_backup;
  integrator_ptr->beta_last_ = beta_backup;
  integrator_ptr->Propagate(dt, acc, gyr);
  std::cout << std::setprecision(9);
  std::cout << "alpha propagation diff: "
            << (integrator_ptr->alpha_curr_ - alpha_propagate).transpose()
            << std::endl;
  std::cout << "alpha Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(0, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "q propagation diff: "
            << ((q_propagate.inverse() * integrator_ptr->q_curr_).vec() * 2)
                   .transpose()
            << std::endl;
  std::cout << "q Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(3, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "beta propagation diff: "
            << (integrator_ptr->beta_curr_ - beta_propagate).transpose()
            << std::endl;
  std::cout << "beta Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(6, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "ba propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "ba Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(9, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "bg propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "bg Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(12, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;

  // ===========================================================================

  // Step 3: Perturb q and check the result.
  PRINT_DEBUG("=============== Perturb q. ================\n");
  integrator_ptr->acc_m_0_ = acc_0_backup;
  integrator_ptr->gyr_m_0_ = gyr_0_backup;
  integrator_ptr->alpha_last_ = alpha_backup;
  integrator_ptr->q_last_ =
      q_backup * Eigen::Quaterniond(1, kPerturbVec[0] / 2., kPerturbVec[1] / 2.,
                                    kPerturbVec[2] / 2.)
                     .normalized();
  integrator_ptr->beta_last_ = beta_backup;
  integrator_ptr->Propagate(dt, acc, gyr);
  std::cout << "alpha propagation diff: "
            << (integrator_ptr->alpha_curr_ - alpha_propagate).transpose()
            << std::endl;
  std::cout << "alpha Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(0, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "q propagation diff: "
            << ((q_propagate.inverse() * integrator_ptr->q_curr_).vec() * 2)
                   .transpose()
            << std::endl;
  std::cout << "q Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(3, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "beta propagation diff: "
            << (integrator_ptr->beta_curr_ - beta_propagate).transpose()
            << std::endl;
  std::cout << "beta Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(6, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "ba propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "ba Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(9, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "bg propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "bg Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(12, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;

  // ===========================================================================

  // Step 4: Perturb beta and check the result.
  PRINT_DEBUG("============== Perturb beta. ==============\n");
  integrator_ptr->acc_m_0_ = acc_0_backup;
  integrator_ptr->gyr_m_0_ = gyr_0_backup;
  integrator_ptr->alpha_last_ = alpha_backup;
  integrator_ptr->q_last_ = q_backup;
  integrator_ptr->beta_last_ = beta_backup + kPerturbVec;
  integrator_ptr->Propagate(dt, acc, gyr);
  std::cout << "alpha propagation diff: "
            << (integrator_ptr->alpha_curr_ - alpha_propagate).transpose()
            << std::endl;
  std::cout << "alpha Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(0, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "q propagation diff: "
            << ((q_propagate.inverse() * integrator_ptr->q_curr_).vec() * 2)
                   .transpose()
            << std::endl;
  std::cout << "q Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(3, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "beta propagation diff: "
            << (integrator_ptr->beta_curr_ - beta_propagate).transpose()
            << std::endl;
  std::cout << "beta Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(6, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "ba propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "ba Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(9, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "bg propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "bg Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(12, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;

  // ===========================================================================

  // Step 5: Perturb ba and check the result.
  PRINT_DEBUG("=============== Perturb ba. ===============\n");
  integrator_ptr->acc_m_0_ = acc_0_backup;
  integrator_ptr->gyr_m_0_ = gyr_0_backup;
  integrator_ptr->alpha_last_ = alpha_backup;
  integrator_ptr->q_last_ = q_backup;
  integrator_ptr->beta_last_ = beta_backup;
  integrator_ptr->linearized_ba_ += kPerturbVec;
  integrator_ptr->Propagate(dt, acc, gyr);
  std::cout << "alpha propagation diff: "
            << (integrator_ptr->alpha_curr_ - alpha_propagate).transpose()
            << std::endl;
  std::cout << "alpha Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(0, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "q propagation diff: "
            << ((q_propagate.inverse() * integrator_ptr->q_curr_).vec() * 2)
                   .transpose()
            << std::endl;
  std::cout << "q Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(3, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "beta propagation diff: "
            << (integrator_ptr->beta_curr_ - beta_propagate).transpose()
            << std::endl;
  std::cout << "beta Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(6, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "ba propagation diff: " << kPerturbVec.transpose() << std::endl;
  std::cout << "ba Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(9, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "bg propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "bg Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(12, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;

  // ===========================================================================

  // Step 6: Perturb bg and check the result.
  PRINT_DEBUG("=============== Perturb bg. ===============\n");
  integrator_ptr->acc_m_0_ = acc_0_backup;
  integrator_ptr->gyr_m_0_ = gyr_0_backup;
  integrator_ptr->alpha_last_ = alpha_backup;
  integrator_ptr->q_last_ = q_backup;
  integrator_ptr->beta_last_ = beta_backup;
  integrator_ptr->linearized_ba_ = ba_backup;
  integrator_ptr->linearized_bg_ += kPerturbVec;
  integrator_ptr->Propagate(dt, acc, gyr);
  std::cout << "alpha propagation diff: "
            << (integrator_ptr->alpha_curr_ - alpha_propagate).transpose()
            << std::endl;
  std::cout << "alpha Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(0, 12) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "q propagation diff: "
            << ((q_propagate.inverse() * integrator_ptr->q_curr_).vec() * 2)
                   .transpose()
            << std::endl;
  std::cout << "q Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(3, 12) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "beta propagation diff: "
            << (integrator_ptr->beta_curr_ - beta_propagate).transpose()
            << std::endl;
  std::cout << "beta Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(6, 12) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "ba propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "ba Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(9, 12) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "bg propagation diff: " << kPerturbVec.transpose() << std::endl;
  std::cout << "bg Jacobian diff   : "
            << (integrator_ptr->step_state_jacobian_.block<3, 3>(12, 12) *
                kPerturbVec)
                   .transpose()
            << std::endl;

  // ===========================================================================

  // Step 7: Perturb acc0 and check the result.
  PRINT_DEBUG("============== Perturb acc0. ==============\n");
  integrator_ptr->acc_m_0_ = acc_0_backup + kPerturbVec;
  integrator_ptr->gyr_m_0_ = gyr_0_backup;
  integrator_ptr->alpha_last_ = alpha_backup;
  integrator_ptr->q_last_ = q_backup;
  integrator_ptr->beta_last_ = beta_backup;
  integrator_ptr->linearized_ba_ = ba_backup;
  integrator_ptr->linearized_bg_ = bg_backup;
  integrator_ptr->Propagate(dt, acc, gyr);
  std::cout << "alpha propagation diff: "
            << (integrator_ptr->alpha_curr_ - alpha_propagate).transpose()
            << std::endl;
  std::cout << "alpha Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(0, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "q propagation diff: "
            << ((q_propagate.inverse() * integrator_ptr->q_curr_).vec() * 2)
                   .transpose()
            << std::endl;
  std::cout << "q Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(3, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "beta propagation diff: "
            << (integrator_ptr->beta_curr_ - beta_propagate).transpose()
            << std::endl;
  std::cout << "beta Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(6, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "ba propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "ba Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(9, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "bg propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "bg Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(12, 0) *
                kPerturbVec)
                   .transpose()
            << std::endl;

  // ===========================================================================

  // Step 8: Perturb gyr0 and check the result.
  PRINT_DEBUG("============== Perturb gyr0. ==============\n");
  integrator_ptr->acc_m_0_ = acc_0_backup;
  integrator_ptr->gyr_m_0_ = gyr_0_backup + kPerturbVec;
  integrator_ptr->alpha_last_ = alpha_backup;
  integrator_ptr->q_last_ = q_backup;
  integrator_ptr->beta_last_ = beta_backup;
  integrator_ptr->Propagate(dt, acc, gyr);
  std::cout << "alpha propagation diff: "
            << (integrator_ptr->alpha_curr_ - alpha_propagate).transpose()
            << std::endl;
  std::cout << "alpha Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(0, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "q propagation diff: "
            << ((q_propagate.inverse() * integrator_ptr->q_curr_).vec() * 2)
                   .transpose()
            << std::endl;
  std::cout << "q Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(3, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "beta propagation diff: "
            << (integrator_ptr->beta_curr_ - beta_propagate).transpose()
            << std::endl;
  std::cout << "beta Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(6, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "ba propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "ba Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(9, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "bg propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "bg Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(12, 3) *
                kPerturbVec)
                   .transpose()
            << std::endl;

  // ===========================================================================

  // Step 9: Perturb acc1 and check the result.
  PRINT_DEBUG("============== Perturb acc1. ==============\n");
  integrator_ptr->acc_m_0_ = acc_0_backup;
  integrator_ptr->gyr_m_0_ = gyr_0_backup;
  integrator_ptr->alpha_last_ = alpha_backup;
  integrator_ptr->q_last_ = q_backup;
  integrator_ptr->beta_last_ = beta_backup;
  integrator_ptr->Propagate(dt, acc + kPerturbVec, gyr);
  std::cout << "alpha propagation diff: "
            << (integrator_ptr->alpha_curr_ - alpha_propagate).transpose()
            << std::endl;
  std::cout << "alpha Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(0, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "q propagation diff: "
            << ((q_propagate.inverse() * integrator_ptr->q_curr_).vec() * 2)
                   .transpose()
            << std::endl;
  std::cout << "q Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(3, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "beta propagation diff: "
            << (integrator_ptr->beta_curr_ - beta_propagate).transpose()
            << std::endl;
  std::cout << "beta Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(6, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "ba propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "ba Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(9, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "bg propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "bg Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(12, 6) *
                kPerturbVec)
                   .transpose()
            << std::endl;

  // ===========================================================================

  // Step 10: Perturb gyr1 and check the result.
  PRINT_DEBUG("============== Perturb gyr1. ==============\n");
  integrator_ptr->acc_m_0_ = acc_0_backup;
  integrator_ptr->gyr_m_0_ = gyr_0_backup;
  integrator_ptr->alpha_last_ = alpha_backup;
  integrator_ptr->q_last_ = q_backup;
  integrator_ptr->beta_last_ = beta_backup;
  integrator_ptr->Propagate(dt, acc, gyr + kPerturbVec);
  std::cout << "alpha propagation diff: "
            << (integrator_ptr->alpha_curr_ - alpha_propagate).transpose()
            << std::endl;
  std::cout << "alpha Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(0, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "q propagation diff: "
            << ((q_propagate.inverse() * integrator_ptr->q_curr_).vec() * 2)
                   .transpose()
            << std::endl;
  std::cout << "q Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(3, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "beta propagation diff: "
            << (integrator_ptr->beta_curr_ - beta_propagate).transpose()
            << std::endl;
  std::cout << "beta Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(6, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "ba propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "ba Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(9, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;
  std::cout << "bg propagation diff: " << Eigen::Vector3d::Zero().transpose()
            << std::endl;
  std::cout << "bg Jacobian diff   : "
            << (integrator_ptr->step_noise_jacobian_.block<3, 3>(12, 9) *
                kPerturbVec)
                   .transpose()
            << std::endl;

  // ===========================================================================
}

int main() {
  mocap2gt::Printer::SetPrintLevel(mocap2gt::Printer::PrintLevel::ALL);

  const Eigen::Vector3d kBa(0.01, 0.01, 0.01);
  const Eigen::Vector3d kBg(0.001, 0.001, 0.001);
  const Eigen::Vector4d kImuNoise(2e-3, 3e-3, 2e-4, 2e-5);
  constexpr double kGravityMagnitude = 9.81;

  // Set up the test data.
  Eigen::Matrix<double, 4, 7> imu_data_example;
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

  // Initialize the IMU preintegrator.
  std::shared_ptr<mocap2gt::ImuPreintegrator> integrator(
      new mocap2gt::ImuPreintegrator(
          imu_data_example.block<1, 3>(0, 4),
          imu_data_example.block<1, 3>(0, 1), kBa, kBg, kImuNoise[0],
          kImuNoise[1], kImuNoise[2], kImuNoise[3], kGravityMagnitude));

  // Perform propagation and test the preintegration result and the Jacobian
  // calculation.
  for (int i = 1; i < imu_data_example.rows(); ++i) {
    double dt = imu_data_example(i, 0) - imu_data_example(i - 1, 0);
    if (i < imu_data_example.rows() - 1) {
      integrator->Propagate(dt, imu_data_example.block<1, 3>(i, 4),
                            imu_data_example.block<1, 3>(i, 1));
    } else {
      mocap2gt::CheckJacobian(integrator.get(), dt,
                              imu_data_example.block<1, 3>(i, 4),
                              imu_data_example.block<1, 3>(i, 1));
    }
  }

  return 0;
}
