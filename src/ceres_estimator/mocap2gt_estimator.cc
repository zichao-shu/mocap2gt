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

#include "mocap2gt/ceres_estimator/mocap2gt_estimator.h"

#include <ctime>

#include "mocap2gt/ceres_estimator/pose_pq_parameterization.h"
#include "mocap2gt/ceres_estimator/rot_rp_parameterization.h"
#include "mocap2gt/initializer/extrinsic_initializer.h"
#include "mocap2gt/initializer/time_offset_initializer.h"

namespace mocap2gt {

MoCap2GTEstimator::MoCap2GTEstimator(
    const std::shared_ptr<ConfigParser> c_parser,
    const std::shared_ptr<ImuMeasManager> i_manager,
    const std::shared_ptr<MoCapMeasManagerBase> m_manager) {
  config_parser_ = c_parser;
  imu_manager_ = i_manager;
  mocap_manager_ = m_manager;

  // Set optimization options
  relin_iter_count_ = config_parser_->get_relin_iter_count();
  est_T_MI_flag_ = config_parser_->get_est_T_MI_flag();
  est_q_WG_flag_ = config_parser_->get_est_q_WG_flag();
  est_toff_MI_flag_ = config_parser_->get_est_toff_MI_flag();
  est_traj_pose_num_ = 0;
}

void MoCap2GTEstimator::BuildAndOptimize() {
  // Step 1: Set the initial guesses of the calibration parameters.
  clock_t start_time = clock();
  InitializeCalibParameters();

  // ===========================================================================

  // Step 2: Obtain the timestamps for the estimated trajectory based on
  // frequency specified by user.
  double imu_start_time = imu_manager_->get_start_time();
  double imu_end_time = imu_manager_->get_end_time();
  double mocap_start_time = mocap_manager_->get_start_time() + para_toff_MI_[0];
  double mocap_end_time = mocap_manager_->get_end_time() + para_toff_MI_[0];

  // Keep the timestamp of the trajectory within the range.
  double est_traj_start_time =
      imu_start_time > mocap_start_time ? imu_start_time : mocap_start_time;
  double est_traj_end_time =
      imu_end_time < mocap_end_time ? imu_end_time : mocap_end_time;

  est_traj_start_time += config_parser_->get_max_toff();
  est_traj_end_time -= config_parser_->get_max_toff();
  double est_traj_duration = est_traj_end_time - est_traj_start_time;

  target_timestamps_.clear();
  if (est_traj_start_time > 0 && est_traj_end_time > 0 &&
      est_traj_start_time < est_traj_end_time) {
    double pose_interval = 1. / config_parser_->get_est_traj_freq();
    double temp_time = est_traj_start_time;
    target_timestamps_.reserve(
        static_cast<int>(est_traj_duration / pose_interval) + 2);

    while (temp_time < est_traj_end_time) {
      target_timestamps_.push_back(temp_time);
      temp_time += pose_interval;
    }
  } else {
    PRINT_ERROR(
        "[Error]: Start/end time of target trajectory in IMU clock exception: "
        "%f/%f. \n",
        est_traj_start_time, est_traj_end_time);
    std::exit(EXIT_FAILURE);
  }

  if (target_timestamps_.size() > 2) {
    PRINT_INFO(
        "[Info]: Start/end time of target trajectory in IMU clock: %f/%f, "
        "contains %d poses. \n",
        est_traj_start_time, est_traj_end_time, target_timestamps_.size());
  } else {
    PRINT_ERROR("[Error]: Insufficient poses for estimated trajectory. \n")

    std::exit(EXIT_FAILURE);
  }

  est_traj_pose_num_ = target_timestamps_.size();
  imu_preinteg_blocks_.reserve(est_traj_pose_num_);

  // ===========================================================================

  // Step 3: Set the initial guesses of the imu states.
  InitialImuStates();

  clock_t end_time = clock();
  double duration = static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC;
  PRINT_INFO("[Info]: Initialization done, took %.3f seconds. \n", duration);

  // ===========================================================================

  // Step 4: Truncate time offset to avoid unstable optimization, and set the
  // variable time offset if needed.
  toff_truncate_ = para_toff_MI_[0];
  if (config_parser_->get_variable_toff_flag()) {
    double toff_control_point_interval =
        config_parser_->get_toff_control_point_interval();
    toff_control_points.clear();
    toff_control_points.emplace_back(target_timestamps_[0]);
    for (const auto &imu_time : target_timestamps_) {
      if ((imu_time - toff_control_points.back()) >
          toff_control_point_interval) {
        toff_control_points.emplace_back(imu_time);
      }
    }

    if (target_timestamps_.back() - toff_control_points.back() <
        toff_control_point_interval / 10.) {
      toff_control_points.pop_back();
    }
    toff_control_points.emplace_back(target_timestamps_.back());

    para_toff_MI_.reset(new double[toff_control_points.size()]);
    std::fill(para_toff_MI_.get(),
              para_toff_MI_.get() + toff_control_points.size(), 0.);
  } else {
    toff_control_points.emplace_back(target_timestamps_[0]);
    para_toff_MI_[0] = 0.;
  }

  // ===========================================================================

  // Step 5: Optimize with relinearization iterations.
  PRINT_INFO(
      "[Info]: Optimization and relinearization for %d rounds, it may take "
      "some time... \n",
      relin_iter_count_);
  for (int i = 0; i <= relin_iter_count_; ++i) {
    // Build the problem.
    BuildProblem(i == 0);

    // Solve the problem.
    PRINT_INFO("[Info]: Round %d: \n", i);
    if (!OptimizeProblem()) {
      PRINT_ERROR("[Error]: Optimization exception, abort. \n")
      break;
    }
  }

  end_time = clock();
  duration = static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC;
  PRINT_INFO("[Info]: Optimization done, took %.3f seconds. \n", duration);

  // ===========================================================================
}

void MoCap2GTEstimator::GetImuStates(TimestampStateVec *time_states_ptr) {
  time_states_ptr->clear();
  time_states_ptr->reserve(est_traj_pose_num_);

  for (int i = 0; i < est_traj_pose_num_; ++i) {
    Eigen::Matrix<double, 17, 1> time_state =
        Eigen::Matrix<double, 17, 1>::Zero();

    double timestamp = target_timestamps_.at(i);
    time_state(0) = timestamp;
    for (int j = 0; j < 7; j++) {
      time_state(j + 1) = para_pose_WI_[i * 7 + j];
    }
    for (int k = 0; k < 9; k++) {
      time_state(k + 8) = para_speed_bias_WI_[i * 9 + k];
    }

    time_states_ptr->push_back(time_state);
  }
}

void MoCap2GTEstimator::GetCalibParameters(std::vector<double> *toff_ptr,
                                           Eigen::Vector3d *p_MI_ptr,
                                           Eigen::Quaterniond *q_MI_ptr,
                                           Eigen::Quaterniond *q_WG_ptr) {
  if (est_traj_pose_num_ == 0) {
    return;
  }

  for (size_t i = 0; i < toff_control_points.size(); i++) {
    toff_ptr->push_back(para_toff_MI_[i] + toff_truncate_);
  }
  *p_MI_ptr << para_extrinsic_MI_[0], para_extrinsic_MI_[1],
      para_extrinsic_MI_[2];
  q_MI_ptr->coeffs() << para_extrinsic_MI_[3], para_extrinsic_MI_[4],
      para_extrinsic_MI_[5], para_extrinsic_MI_[6];
  Eigen::Vector3d rpy_WG(para_rotxy_WG_[0], para_rotxy_WG_[1], 0.);
  *q_WG_ptr = Rpy2Quat(rpy_WG);
}

void MoCap2GTEstimator::InitializeCalibParameters() {
  // Allocate memory of the parameters.
  para_toff_MI_.reset(new double[1]);
  para_extrinsic_MI_.reset(new double[7]);
  para_rotxy_WG_.reset(new double[2]);

  // ===========================================================================

  // Initialize time offset.
  double toff_MI_init = 0.;
  if (config_parser_->get_use_initializer_flag() &&
      config_parser_->get_est_toff_MI_flag()) {
    std::shared_ptr<mocap2gt::TimeOffsetInitializer> toff_initializer(
        new mocap2gt::TimeOffsetInitializer(imu_manager_, mocap_manager_,
                                            config_parser_->get_max_toff()));
    toff_initializer->EstimateToff(&toff_MI_init);
    PRINT_INFO(
        "[Info]: The initial guess of toff_MI estimated by initializer is: "
        "%.6f\n",
        toff_MI_init);
  } else {
    toff_MI_init = config_parser_->get_toff_MI_init();
  }

  // ===========================================================================

  // Initialize spatial extrinsic.
  Eigen::Vector3d p_MI_init = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_MI_init = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_WG_init = Eigen::Quaterniond::Identity();
  Eigen::Vector3d rpy_WG_init = Eigen::Vector3d::Zero();
  if (config_parser_->get_use_initializer_flag() &&
      (config_parser_->get_est_T_MI_flag() ||
       config_parser_->get_est_q_WG_flag())) {
    std::shared_ptr<mocap2gt::ExtrinsicInitializer> ex_initializer(
        new mocap2gt::ExtrinsicInitializer(imu_manager_, mocap_manager_,
                                           config_parser_->get_max_toff(),
                                           toff_MI_init));
    ex_initializer->EstimateExtrinsic(&p_MI_init, &q_MI_init, &q_WG_init,
                                      config_parser_->get_gravity_magnitude());
    if (config_parser_->get_est_T_MI_flag()) {
      PRINT_INFO(
          "[Info]: The initial guess of p_MI estimated by initializer is: "
          "[%.6f, %.6f, %.6f]\n",
          p_MI_init.x(), p_MI_init.y(), p_MI_init.z());
      PRINT_INFO(
          "[Info]: The initial guess of q_MI estimated by initializer is: "
          "[%.6f, %.6f, %.6f, %.6f]\n",
          q_MI_init.x(), q_MI_init.y(), q_MI_init.z(), q_MI_init.w());
    } else {
      p_MI_init = config_parser_->get_p_MI_init();
      q_MI_init = config_parser_->get_q_MI_init();
    }
    if (config_parser_->get_est_q_WG_flag()) {
      PRINT_INFO(
          "[Info]: The initial guess of q_WG estimated by initializer is: "
          "[%.6f, %.6f, %.6f, %.6f]\n",
          q_WG_init.x(), q_WG_init.y(), q_WG_init.z(), q_WG_init.w());
      rpy_WG_init = Quat2Rpy(q_WG_init);
    } else {
      rpy_WG_init = Quat2Rpy(config_parser_->get_q_WG_init());
    }
  } else {
    p_MI_init = config_parser_->get_p_MI_init();
    q_MI_init = config_parser_->get_q_MI_init();
    rpy_WG_init = Quat2Rpy(config_parser_->get_q_WG_init());
  }

  // ===========================================================================

  // Set the values.
  para_toff_MI_[0] = toff_MI_init;
  para_extrinsic_MI_[0] = p_MI_init.x();
  para_extrinsic_MI_[1] = p_MI_init.y();
  para_extrinsic_MI_[2] = p_MI_init.z();
  para_extrinsic_MI_[3] = q_MI_init.x();
  para_extrinsic_MI_[4] = q_MI_init.y();
  para_extrinsic_MI_[5] = q_MI_init.z();
  para_extrinsic_MI_[6] = q_MI_init.w();
  para_rotxy_WG_[0] = rpy_WG_init.x();
  para_rotxy_WG_[1] = rpy_WG_init.y();

  // ===========================================================================
}

void MoCap2GTEstimator::InitialImuStates() {
  // Allocate memory of the states.
  para_pose_WI_.reset(new double[est_traj_pose_num_ * 7]);
  para_speed_bias_WI_.reset(new double[est_traj_pose_num_ * 9]);

  // Set the initial guesses of the IMU states.
  double toff_MI_init = para_toff_MI_[0];
  Eigen::Vector3d p_MI_init(para_extrinsic_MI_[0], para_extrinsic_MI_[1],
                            para_extrinsic_MI_[2]);
  Eigen::Quaterniond q_MI_init(para_extrinsic_MI_[6], para_extrinsic_MI_[3],
                               para_extrinsic_MI_[4], para_extrinsic_MI_[5]);
  for (int i = 0; i < est_traj_pose_num_; ++i) {
    // Convert the IMU timestamp to the Mocap clock and retrieve the pose.
    // We also obtain the poses before and after to calculate the initial
    // velocity.
    double mocap_time_before = target_timestamps_.at(i) - toff_MI_init -
                               config_parser_->get_vel_init_range() / 2;
    double mocap_time_current = target_timestamps_.at(i) - toff_MI_init;
    double mocap_time_after = target_timestamps_.at(i) - toff_MI_init +
                              config_parser_->get_vel_init_range() / 2;

    Eigen::Vector3d p_WM_before = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_WM_current = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_WM_after = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_WM_before = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_WM_current = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_WM_after = Eigen::Quaterniond::Identity();

    bool has_mocap_before =
        mocap_manager_->GetPose(mocap_time_before, &p_WM_before, &q_WM_before);
    bool has_mocap_current = mocap_manager_->GetPose(
        mocap_time_current, &p_WM_current, &q_WM_current);
    bool has_mocap_after =
        mocap_manager_->GetPose(mocap_time_after, &p_WM_after, &q_WM_after);

    if (!has_mocap_before || !has_mocap_current || !has_mocap_after) {
      // If MoCap measurements are unavailable, set the initial guess to 0.
      para_pose_WI_[i * 7] = 0.;
      para_pose_WI_[i * 7 + 1] = 0.;
      para_pose_WI_[i * 7 + 2] = 0.;
      para_pose_WI_[i * 7 + 3] = 0.;
      para_pose_WI_[i * 7 + 4] = 0.;
      para_pose_WI_[i * 7 + 5] = 0.;
      para_pose_WI_[i * 7 + 6] = 0.;

      para_speed_bias_WI_[i * 9] = 0.;
      para_speed_bias_WI_[i * 9 + 1] = 0.;
      para_speed_bias_WI_[i * 9 + 2] = 0.;
      para_speed_bias_WI_[i * 9 + 3] = 0.;
      para_speed_bias_WI_[i * 9 + 4] = 0.;
      para_speed_bias_WI_[i * 9 + 5] = 0.;
      para_speed_bias_WI_[i * 9 + 6] = 0.;
      para_speed_bias_WI_[i * 9 + 7] = 0.;
      para_speed_bias_WI_[i * 9 + 8] = 0.;

      PRINT_DEBUG(
          "[Debug]: Fail to set the initial value, incomplete MoCap "
          "measurements within the time (%.6f, %.6f). \n",
          mocap_time_before, mocap_time_after);

      continue;
    }

    Eigen::Vector3d p_WI_before = q_WM_before * p_MI_init + p_WM_before;
    Eigen::Vector3d p_WI_current = q_WM_current * p_MI_init + p_WM_current;
    Eigen::Vector3d p_WI_after = q_WM_after * p_MI_init + p_WM_after;
    Eigen::Quaterniond q_WI_current = q_WM_current * q_MI_init;

    // Calculate the initial velocity.
    Eigen::Vector3d v_WI_current =
        (p_WI_after - p_WI_before) / config_parser_->get_vel_init_range();

    // Set the initial guesses of the states.
    para_pose_WI_[i * 7] = p_WI_current.x();
    para_pose_WI_[i * 7 + 1] = p_WI_current.y();
    para_pose_WI_[i * 7 + 2] = p_WI_current.z();
    para_pose_WI_[i * 7 + 3] = q_WI_current.x();
    para_pose_WI_[i * 7 + 4] = q_WI_current.y();
    para_pose_WI_[i * 7 + 5] = q_WI_current.z();
    para_pose_WI_[i * 7 + 6] = q_WI_current.w();

    para_speed_bias_WI_[i * 9] = v_WI_current.x();
    para_speed_bias_WI_[i * 9 + 1] = v_WI_current.y();
    para_speed_bias_WI_[i * 9 + 2] = v_WI_current.z();
    para_speed_bias_WI_[i * 9 + 3] = 0.;
    para_speed_bias_WI_[i * 9 + 4] = 0.;
    para_speed_bias_WI_[i * 9 + 5] = 0.;
    para_speed_bias_WI_[i * 9 + 6] = 0.;
    para_speed_bias_WI_[i * 9 + 7] = 0.;
    para_speed_bias_WI_[i * 9 + 8] = 0.;
  }
}

void MoCap2GTEstimator::BuildProblem(bool first_relin) {
  // Memory will be freed by ceres
  ceres::LocalParameterization *pose_pq_param = new PosePQParameterization();
  ceres::LocalParameterization *rot_rp_param = new RotRPParameterization();

  // ===========================================================================

  // Add the spatiotemporal calibration parameter block and decide whether to
  // fix them. Only performed in the first linearization.
  if (first_relin) {
    problem_.AddParameterBlock(para_extrinsic_MI_.get(), 7, pose_pq_param);
    problem_.AddParameterBlock(para_rotxy_WG_.get(), 2, rot_rp_param);

    if (!est_T_MI_flag_) {
      problem_.SetParameterBlockConstant(para_extrinsic_MI_.get());
      PRINT_INFO("[Info]: Fix extrinsic T_MI in optimization. \n");
    }

    if (!est_q_WG_flag_) {
      problem_.SetParameterBlockConstant(para_rotxy_WG_.get());
      PRINT_INFO("[Info]: Fix gravity alignment q_WG in optimization. \n");
    }

    if (!est_toff_MI_flag_) {
      problem_.SetParameterBlockConstant(para_toff_MI_.get());
      PRINT_INFO("[Info]: Fix offset toff_MI in optimization. \n");
    }
  }

  // For each relinearization, we will remove and update the residual block of
  // IMU preintegration.
  for (const auto &imu_preinteg_block : imu_preinteg_blocks_) {
    problem_.RemoveResidualBlock(imu_preinteg_block);
  }
  imu_preinteg_blocks_.clear();

  // ===========================================================================

  // Add residual block of IMU preintegration and MoCap interpolation.
  for (size_t i = 0; i < target_timestamps_.size(); ++i) {
    double time_curr = target_timestamps_[i];

    // Add the IMU states parameter block, and add the residual block of MoCap
    // interpolation. Only performed in the first linearization.
    if (first_relin) {
      problem_.AddParameterBlock(para_pose_WI_.get() + i * 7, 7, pose_pq_param);

      // Add the MoCap factor depend on whether the time offset is variable.
      if (config_parser_->get_variable_toff_flag()) {
        if (time_curr > toff_control_points.back() ||
            time_curr < toff_control_points.front()) {
          PRINT_ERROR(
              "[Error]: Unable to find control points for variable time "
              "offset. \n");
        }

        // Find the time offset control points.
        auto iter = std::upper_bound(toff_control_points.begin(),
                                     toff_control_points.end(), time_curr);
        if (iter == toff_control_points.end()) {
          --iter;
        }
        int index = iter - toff_control_points.begin();
        double toff_control_point_m = toff_control_points[index - 1];
        double toff_control_point_n = toff_control_points[index];

        // Construct the MoCap factor with variable time offset and add to the
        // problem.
        MoCapInterpolationVariableFactor *mocap_interp_variable_factor =
            new MoCapInterpolationVariableFactor(
                mocap_manager_, time_curr, toff_control_point_m,
                toff_control_point_n, toff_truncate_);
        problem_.AddResidualBlock(
            mocap_interp_variable_factor, NULL, para_pose_WI_.get() + i * 7,
            para_extrinsic_MI_.get(), para_toff_MI_.get() + index - 1,
            para_toff_MI_.get() + index);
      } else {
        // Construct the MoCap factor with fixed time offset and add to the
        // problem.
        MoCapInterpolationFactor *mocap_interp_factor =
            new MoCapInterpolationFactor(mocap_manager_, time_curr,
                                         toff_truncate_);
        problem_.AddResidualBlock(
            mocap_interp_factor, NULL, para_pose_WI_.get() + i * 7,
            para_extrinsic_MI_.get(), para_toff_MI_.get());
      }
    }

    if (i == 0) {
      continue;
    }

    // Add the residual block of IMU preintegration.
    double time_0 = target_timestamps_[i - 1];
    Eigen::Map<const Eigen::Vector3d> ba_0(para_speed_bias_WI_.get() +
                                           (i - 1) * 9 + 3);
    Eigen::Map<const Eigen::Vector3d> bg_0(para_speed_bias_WI_.get() +
                                           (i - 1) * 9 + 6);

    std::shared_ptr<ImuPreintegrator> integrator;
    if (imu_manager_->Preintegrate(time_0, time_curr, ba_0, bg_0,
                                   &integrator)) {
      IMUPreintegrationFactor *imu_preinteg_factor =
          new IMUPreintegrationFactor(integrator);
      ceres::ResidualBlockId imu_preinteg_block = problem_.AddResidualBlock(
          imu_preinteg_factor, NULL, para_pose_WI_.get() + (i - 1) * 7,
          para_speed_bias_WI_.get() + (i - 1) * 9, para_pose_WI_.get() + i * 7,
          para_speed_bias_WI_.get() + i * 9, para_rotxy_WG_.get());
      imu_preinteg_blocks_.push_back(imu_preinteg_block);
    }
  }

  // ===========================================================================
}

bool MoCap2GTEstimator::OptimizeProblem() {
  // Options for the ceres solver.
  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.num_threads = 1;
  options.max_num_iterations = 20;
  options.minimizer_progress_to_stdout = true;

  // Solve the problem.
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);

  return summary.IsSolutionUsable();
}

}  // namespace mocap2gt
