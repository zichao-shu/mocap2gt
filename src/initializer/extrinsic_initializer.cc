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

#include "mocap2gt/initializer/extrinsic_initializer.h"

namespace mocap2gt {

ExtrinsicInitializer::ExtrinsicInitializer(
    const std::shared_ptr<ImuMeasManager> i_manager,
    const std::shared_ptr<MoCapMeasManagerBase> m_manager,
    const double& cut_off_time, const double& toff_MI) {
  imu_manager_ = i_manager;
  mocap_manager_ = m_manager;
  toff_MI_ = toff_MI;

  // Set the start and end times of measurements after truncate, and convert IMU
  // time using the time offset result.
  imu_start_time_ = imu_manager_->get_start_time() + cut_off_time - toff_MI;
  imu_end_time_ = imu_manager_->get_end_time() - cut_off_time - toff_MI;
  mocap_start_time_ = mocap_manager_->get_start_time() + cut_off_time;
  mocap_end_time_ = mocap_manager_->get_end_time() - cut_off_time;

  if (imu_start_time_ > imu_end_time_) {
    PRINT_ERROR(
        "[Error]: Start/end time of imu data in extrinsic initializer "
        "exception: %f/%f. \n",
        imu_start_time_, imu_end_time_);
    std::exit(EXIT_FAILURE);
  }

  if (mocap_start_time_ > mocap_end_time_) {
    PRINT_ERROR(
        "[Error]: Start/end time of mocap data in extrinsic initializer "
        "exception: %f/%f. "
        "\n",
        mocap_start_time_, mocap_end_time_);
    std::exit(EXIT_FAILURE);
  }
}

bool ExtrinsicInitializer::EstimateExtrinsic(
    Eigen::Vector3d* p_MI_ptr, Eigen::Quaterniond* q_MI_ptr,
    Eigen::Quaterniond* q_WG_ptr, const double& gravity_magnitude,
    const double& max_sample_interval, const double& element_trans_thresh,
    const double& element_rot_thresh, const int& max_sample_num) {
  // Set the default values.
  *p_MI_ptr = Eigen::Vector3d::Zero();
  *q_MI_ptr = Eigen::Quaterniond::Identity();
  *q_WG_ptr = Eigen::Quaterniond::Identity();

  // ===========================================================================

  // Step 1: Construct the solver elements from IMU and MoCap measurements.
  double joint_start_time =
      imu_start_time_ > mocap_start_time_ ? imu_start_time_ : mocap_start_time_;
  double joint_end_time =
      imu_end_time_ < mocap_end_time_ ? imu_end_time_ : mocap_end_time_;
  if (joint_start_time < 0 || joint_end_time < 0 ||
      joint_start_time > joint_end_time) {
    PRINT_ERROR(
        "[Error]: Start/end time of solver elements data in extrinsic "
        "initializer exception: %f/%f. \n",
        mocap_start_time_, mocap_end_time_);
    std::exit(EXIT_FAILURE);
  }

  // Iterate through MoCap measurements and construct the solver elements.
  std::vector<ExtrinsicSolverElement> solver_elements;
  MoCapMeasManagerBase::TimestampSE3Map mocap_traj_map =
      mocap_manager_->get_traj_points_map();

  auto reference_pose_ptr = mocap_traj_map.begin();
  double reference_time, current_time, delta_time;
  Eigen::Quaterniond reference_q, current_q, delta_q;
  Eigen::Vector3d reference_p, current_p, delta_p;
  double delta_translation, delta_rotation;
  int high_quality_num = 0, low_quality_num = 0;
  for (auto pose_iter = mocap_traj_map.begin();
       pose_iter != mocap_traj_map.end(); ++pose_iter) {
    current_time = pose_iter->first;
    if (current_time < joint_start_time) {
      reference_pose_ptr = pose_iter;
      continue;
    } else if (current_time > joint_end_time) {
      break;
    }

    // Reference and current poses.
    reference_time = reference_pose_ptr->first;
    reference_p = Eigen::Vector3d(reference_pose_ptr->second.block<3, 1>(0, 3));
    reference_q =
        Eigen::Quaterniond(reference_pose_ptr->second.block<3, 3>(0, 0));
    current_p = Eigen::Vector3d(pose_iter->second.block<3, 1>(0, 3));
    current_q = Eigen::Quaterniond(pose_iter->second.block<3, 3>(0, 0));

    // Calculate differences.
    delta_time = current_time - reference_time;
    delta_p = reference_q.inverse() * (current_p - reference_p);
    delta_q = reference_q.inverse() * current_q;
    delta_translation = delta_p.norm();
    delta_rotation = QuatAngleDegree(delta_q);

    // Construct solver elements when the forward search meets the conditions.
    if (delta_time < max_sample_interval / 3) {
      continue;
    } else if (delta_translation > element_trans_thresh ||
               delta_rotation > element_rot_thresh ||
               delta_time > max_sample_interval) {
      std::shared_ptr<ImuPreintegrator> integrator;
      // IMU preintegration.
      imu_manager_->Preintegrate(
          reference_time + toff_MI_, current_time + toff_MI_,
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), &integrator);
      // Construct the solver element.
      ExtrinsicSolverElement solver_element(integrator, reference_p,
                                            reference_q, current_p, current_q,
                                            delta_time);
      // Determine if it is a high-quality element.
      if (delta_translation > element_trans_thresh ||
          delta_time > max_sample_interval) {
        solver_element.high_quality_flag = false;
        low_quality_num++;
      } else {
        solver_element.high_quality_flag = true;
        high_quality_num++;
      }
      solver_elements.push_back(solver_element);
      reference_pose_ptr = pose_iter;
    }
  }

  // ===========================================================================

  // Step 2: Select the solver elements.
  constexpr int kMinElementNum = 100;
  if ((high_quality_num + low_quality_num) < kMinElementNum) {
    PRINT_WARNING(
        "[Warning]: Insufficient extrinsic solver elements for initializing "
        "the extrinsic. Please increase the motion duration and motion "
        "stimuli. \n");
    return false;
  }
  if (low_quality_num > high_quality_num) {
    PRINT_WARNING(
        "[Warning]: Insufficient motion stimuli in extrinsic "
        "initialization, may lead to inaccurate calibration results, please "
        "perform more rapid rotation. \n");
  }

  // We need to control the scale of the linear solver and prioritize
  // high-quality solver elements.
  int sample_num = 0;
  std::vector<ExtrinsicSolverElement> target_elements;
  target_elements.reserve(max_sample_num);
  for (auto& element : solver_elements) {
    if (sample_num >= max_sample_num) {
      break;
    }
    if (element.high_quality_flag) {
      target_elements.push_back(element);
      sample_num++;
    }
  }
  for (auto& element : solver_elements) {
    if (sample_num >= max_sample_num) {
      break;
    }
    if (!element.high_quality_flag) {
      target_elements.push_back(element);
      sample_num++;
    }
  }

  // ===========================================================================

  // Step 3: Build the linear solving system based on selected solver elements
  // to calculate the initial guesses of extrinsic.
  Eigen::Vector3d p_MI_result = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_MI_result = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gravity_in_W_result = Eigen::Vector3d::Zero();
  // Initialize the rotational extrinsic.
  EstimateRotExtrinsic(target_elements, &q_MI_result);
  // Simultaneously initialize the translational extrinsic and the gravity
  // represented in W.
  EstimateTransExtrinsicGravityAlign(target_elements, q_MI_result, &p_MI_result,
                                     &gravity_in_W_result);
  // Refine the initial guesses using the constraint of gravity magnitude.
  RefineGravityAlign(target_elements, q_MI_result, &p_MI_result,
                     &gravity_in_W_result, gravity_magnitude);

  *p_MI_ptr = p_MI_result;
  *q_MI_ptr = q_MI_result;
  *q_WG_ptr = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitZ(), gravity_in_W_result.normalized());

  // ===========================================================================

  return true;
}

void ExtrinsicInitializer::EstimateRotExtrinsic(
    const std::vector<ExtrinsicSolverElement>& solver_elements,
    Eigen::Quaterniond* q_MI_ptr, const int& robust_coeff) {
  // Construct the over-determined equations for solving the rotational
  // extrinsic.
  Eigen::MatrixXd solver_A(solver_elements.size() * 4, 4);
  solver_A.setZero();
  int matrix_index = 0;
  for (auto& element : solver_elements) {
    Eigen::Quaterniond q_Ii_Ij = element.preintegrator->get_q_curr();
    Eigen::Quaterniond q_Mi_Mj = element.q_W_Mi.inverse() * element.q_W_Mj;
    double screw_congruence_factor =
        QuatAngleDegree(q_Ii_Ij) / QuatAngleDegree(q_Mi_Mj);
    if (screw_congruence_factor < 1) {
      screw_congruence_factor = 1. / screw_congruence_factor;
    }
    // Robust kernal based on screw congruence theorem, refer to formula (3).
    double robust_kernel =
        1. / std::exp(robust_coeff * (screw_congruence_factor - 1));

    // Sub-matrix corresponding to the current solver element, refer to formula
    // (2)-(4).
    solver_A.block<4, 4>(matrix_index * 4, 0) =
        robust_kernel * (QuatLeftProd(q_Mi_Mj) - QuatRightProd(q_Ii_Ij));

    matrix_index++;
  }

  // Utilize the SVD algorithm to solve the homogeneous linear least squares
  // problem.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      solver_A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector4d result_q_MI = svd.matrixV().col(3);
  *q_MI_ptr = Eigen::Quaterniond(result_q_MI(0), result_q_MI(1), result_q_MI(2),
                                 result_q_MI(3));
}

void ExtrinsicInitializer::EstimateTransExtrinsicGravityAlign(
    const std::vector<ExtrinsicSolverElement>& solver_elements,
    const Eigen::Quaterniond& q_MI, Eigen::Vector3d* p_MI_ptr,
    Eigen::Vector3d* gravity_ptr) {
  // The total dimension of the states to be solved.
  int state_dim = 3 * (solver_elements.size() + 1) + 3 * 2;

  // Construct the linear solving system for initializing the translational
  // extrinsic and the gravity in W.
  Eigen::MatrixXd solver_A(state_dim, state_dim);
  solver_A.setZero();
  Eigen::VectorXd solver_b(state_dim);
  solver_b.setZero();
  int matrix_index = 0;
  for (auto& element : solver_elements) {
    double dt = element.delta_time;
    Eigen::Matrix3d R_MI = q_MI.toRotationMatrix();
    Eigen::Matrix3d R_W_Mi = element.q_W_Mi.toRotationMatrix();
    Eigen::Matrix3d R_W_Mj = element.q_W_Mj.toRotationMatrix();
    Eigen::Matrix3d R_W_Ii = R_W_Mi * R_MI;

    // Sub-matrix corresponding to the current solver element, refer to formula
    // (5)-(9).
    Eigen::Matrix<double, 6, 12> single_A;
    single_A.setZero();
    Eigen::Matrix<double, 6, 1> single_b;
    single_b.setZero();

    single_A.block<3, 3>(0, 0) = -R_W_Ii.inverse() * dt;
    single_A.block<3, 3>(0, 6) = 1. / 2 * R_W_Ii.inverse() * dt * dt;
    single_A.block<3, 3>(0, 9) = R_W_Ii.inverse() * R_W_Mj - R_MI.inverse();
    single_A.block<3, 3>(3, 0) = -R_W_Ii.inverse();
    single_A.block<3, 3>(3, 3) = R_W_Ii.inverse();
    single_A.block<3, 3>(3, 6) = R_W_Ii.inverse() * dt;

    single_b.block<3, 1>(0, 0) = element.preintegrator->get_alpha_curr() -
                                 R_W_Ii.inverse() * element.p_W_Mj +
                                 R_W_Ii.inverse() * element.p_W_Mi;
    single_b.block<3, 1>(3, 0) = element.preintegrator->get_beta_curr();

    // Provide confidence using the covariance of preintegration.
    Eigen::Matrix<double, 6, 6> meas_cov;
    meas_cov.setZero();
    meas_cov.block<3, 3>(0, 0) =
        element.preintegrator->get_meas_covariance().block<3, 3>(0, 0);
    meas_cov.block<3, 3>(0, 3) =
        element.preintegrator->get_meas_covariance().block<3, 3>(0, 6);
    meas_cov.block<3, 3>(3, 0) =
        element.preintegrator->get_meas_covariance().block<3, 3>(6, 0);
    meas_cov.block<3, 3>(3, 3) =
        element.preintegrator->get_meas_covariance().block<3, 3>(6, 6);

    Eigen::Matrix<double, 12, 12> At_A =
        single_A.transpose() * meas_cov.inverse() * single_A;
    Eigen::Matrix<double, 12, 1> At_b =
        single_A.transpose() * meas_cov.inverse() * single_b;

    // Fill the sub-matrix into the corresponding block.
    solver_A.block<6, 6>(matrix_index * 3, matrix_index * 3) +=
        At_A.topLeftCorner<6, 6>();
    solver_A.bottomRightCorner<6, 6>() += At_A.bottomRightCorner<6, 6>();
    solver_A.block<6, 6>(matrix_index * 3, state_dim - 6) +=
        At_A.topRightCorner<6, 6>();
    solver_A.block<6, 6>(state_dim - 6, matrix_index * 3) +=
        At_A.bottomLeftCorner<6, 6>();

    solver_b.segment<6>(matrix_index * 3) += At_b.topLeftCorner<6, 1>();
    solver_b.tail<6>() += At_b.bottomRightCorner<6, 1>();

    ++matrix_index;
  }

  // Solve the non-homogeneous linear least squares problem.
  Eigen::VectorXd solver_x(state_dim);
  solver_x = solver_A.ldlt().solve(solver_b);

  *p_MI_ptr = solver_x.tail<3>();
  *gravity_ptr = solver_x.segment<3>(state_dim - 6);
}

void ExtrinsicInitializer::RefineGravityAlign(
    const std::vector<ExtrinsicSolverElement>& solver_elements,
    const Eigen::Quaterniond& q_MI, Eigen::Vector3d* p_MI_ptr,
    Eigen::Vector3d* gravity_ptr, const double& gravity_magnitude,
    const int& iter_num) {
  // The total dimension of the states to be solved.
  int state_dim = 3 * (solver_elements.size() + 1) + 2 + 3;
  // Gravity vector constrained by magnitude.
  Eigen::Vector3d standard_gravity =
      gravity_ptr->normalized() * gravity_magnitude;

  // Iterate and refine the translational extrinsic and the gravity in W.
  Eigen::MatrixXd solver_A(state_dim, state_dim);
  solver_A.setZero();
  Eigen::VectorXd solver_b(state_dim);
  solver_b.setZero();
  Eigen::VectorXd solver_x(state_dim);
  solver_x.setZero();
  int matrix_index = 0;
  for (int i = 0; i < iter_num; ++i) {
    // Calculate the tangent space basis of the current gravity vector.
    Eigen::Matrix<double, 3, 2> tangent_basis =
        GravityTangentBasis(standard_gravity);
    solver_A.setZero();
    solver_b.setZero();
    matrix_index = 0;
    for (auto& element : solver_elements) {
      double dt = element.delta_time;
      Eigen::Matrix3d R_MI = q_MI.toRotationMatrix();
      Eigen::Matrix3d R_W_Mi = element.q_W_Mi.toRotationMatrix();
      Eigen::Matrix3d R_W_Mj = element.q_W_Mj.toRotationMatrix();
      Eigen::Matrix3d R_W_Ii = R_W_Mi * R_MI;

      // Sub-matrix corresponding to the current solver element.
      Eigen::Matrix<double, 6, 11> single_A;
      single_A.setZero();
      Eigen::Matrix<double, 6, 1> single_b;
      single_b.setZero();

      single_A.block<3, 3>(0, 0) = -R_W_Ii.inverse() * dt;
      single_A.block<3, 2>(0, 6) =
          1. / 2 * R_W_Ii.inverse() * tangent_basis * dt * dt;
      single_A.block<3, 3>(0, 8) = R_W_Ii.inverse() * R_W_Mj - R_MI.inverse();
      single_A.block<3, 3>(3, 0) = -R_W_Ii.inverse();
      single_A.block<3, 3>(3, 3) = R_W_Ii.inverse();
      single_A.block<3, 2>(3, 6) = R_W_Ii.inverse() * tangent_basis * dt;

      single_b.block<3, 1>(0, 0) =
          element.preintegrator->get_alpha_curr() -
          R_W_Ii.inverse() * element.p_W_Mj +
          R_W_Ii.inverse() * element.p_W_Mi -
          1. / 2 * R_W_Ii.inverse() * standard_gravity * dt * dt;
      single_b.block<3, 1>(3, 0) = element.preintegrator->get_beta_curr() -
                                   R_W_Ii.inverse() * standard_gravity * dt;

      // Provide confidence using the covariance of preintegration.
      Eigen::Matrix<double, 6, 6> meas_cov;
      meas_cov.setZero();
      meas_cov.block<3, 3>(0, 0) =
          element.preintegrator->get_meas_covariance().block<3, 3>(0, 0);
      meas_cov.block<3, 3>(0, 3) =
          element.preintegrator->get_meas_covariance().block<3, 3>(0, 6);
      meas_cov.block<3, 3>(3, 0) =
          element.preintegrator->get_meas_covariance().block<3, 3>(6, 0);
      meas_cov.block<3, 3>(3, 3) =
          element.preintegrator->get_meas_covariance().block<3, 3>(6, 6);

      // Fill the sub-matrix into the corresponding block.
      Eigen::Matrix<double, 11, 11> At_A =
          single_A.transpose() * meas_cov.inverse() * single_A;
      Eigen::Matrix<double, 11, 1> At_b =
          single_A.transpose() * meas_cov.inverse() * single_b;

      solver_A.block<6, 6>(matrix_index * 3, matrix_index * 3) +=
          At_A.topLeftCorner<6, 6>();
      solver_A.bottomRightCorner<5, 5>() += At_A.bottomRightCorner<5, 5>();
      solver_A.block<6, 5>(matrix_index * 3, state_dim - 5) +=
          At_A.topRightCorner<6, 5>();
      solver_A.block<5, 6>(state_dim - 5, matrix_index * 3) +=
          At_A.bottomLeftCorner<5, 6>();

      solver_b.segment<6>(matrix_index * 3) += At_b.topLeftCorner<6, 1>();
      solver_b.tail<5>() += At_b.bottomRightCorner<5, 1>();

      ++matrix_index;
    }

    // Solve the non-homogeneous linear least squares problem.
    solver_x = solver_A.ldlt().solve(solver_b);

    // Coefficients of the tangent space basis.
    Eigen::Vector2d basis_delta = solver_x.segment<2>(state_dim - 5);
    // Update the gravity.
    standard_gravity =
        (standard_gravity + tangent_basis * basis_delta).normalized() *
        gravity_magnitude;
  }

  *p_MI_ptr = solver_x.tail<3>();
  *gravity_ptr = standard_gravity;
}

Eigen::Matrix<double, 3, 2> ExtrinsicInitializer::GravityTangentBasis(
    const Eigen::Vector3d& gravity) {
  Eigen::Vector3d normalized_gravity = gravity.normalized();
  Eigen::Vector3d basis_0, basis_1;
  Eigen::Vector3d basis_temp = Eigen::Vector3d::UnitZ();
  if (normalized_gravity == basis_temp) {
    basis_temp = Eigen::Vector3d::UnitX();
  }

  // Calculate basis 0 perpendicular to normalized gravity.
  basis_0 = (basis_temp -
             normalized_gravity * (normalized_gravity.transpose() * basis_temp))
                .normalized();
  // basis 1 = normalized gravity × basis 0
  basis_1 = normalized_gravity.cross(basis_0);

  Eigen::Matrix<double, 3, 2> basis;
  basis.block<3, 1>(0, 0) = basis_0;
  basis.block<3, 1>(0, 1) = basis_1;

  return basis;
}

}  // namespace mocap2gt
