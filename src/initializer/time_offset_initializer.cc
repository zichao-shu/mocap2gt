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

#include "mocap2gt/initializer/time_offset_initializer.h"

#include <numeric>

namespace mocap2gt {

TimeOffsetInitializer::TimeOffsetInitializer(
    const std::shared_ptr<ImuMeasManager>& i_manager,
    const std::shared_ptr<MoCapMeasManagerBase>& m_manager,
    const double& truncate_time) {
  imu_manager_ = i_manager;
  mocap_manager_ = m_manager;

  // Set the start and end times of measurements after truncate.
  imu_start_time_ = imu_manager_->get_start_time() + truncate_time;
  imu_end_time_ = imu_manager_->get_end_time() - truncate_time;
  mocap_start_time_ = mocap_manager_->get_start_time() + truncate_time;
  mocap_end_time_ = mocap_manager_->get_end_time() - truncate_time;

  if (imu_start_time_ < 0 || imu_end_time_ < 0 ||
      imu_start_time_ > imu_end_time_) {
    PRINT_ERROR(
        "[Error]: Start/end time of imu data in time offset initializer "
        "exception: %f/%f. \n",
        imu_start_time_, imu_end_time_);
    std::exit(EXIT_FAILURE);
  }

  if (mocap_start_time_ < 0 || mocap_end_time_ < 0 ||
      mocap_start_time_ > mocap_end_time_) {
    PRINT_ERROR(
        "[Error]: Start/end time of mocap data in time offset initializer "
        "exception: %f/%f. \n",
        mocap_start_time_, mocap_end_time_);
    std::exit(EXIT_FAILURE);
  }
}

bool TimeOffsetInitializer::EstimateToff(double* toff_MI_ptr,
                                         const double& sample_interval,
                                         const int& smooth_kernal_size) {
  *toff_MI_ptr = 0;
  double imu_duration = imu_end_time_ - imu_start_time_;
  double mocap_duration = mocap_end_time_ - mocap_start_time_;

  // ===========================================================================

  // Obtain sampling timestamps based on sampling interval, and calculate the 3D
  // angular velocity.
  int imu_sample_num, mocap_sample_num;
  std::vector<double> imu_sample_timestamps;
  Eigen::Matrix<double, 3, Eigen::Dynamic> imu_angular_vel_3d;
  std::vector<double> mocap_sample_timestamps;
  Eigen::Matrix<double, 3, Eigen::Dynamic> mocap_angular_vel_3d;

  // IMU angular velocity
  imu_sample_num = static_cast<int>(imu_duration / sample_interval);
  imu_sample_timestamps.reserve(imu_sample_num);
  imu_angular_vel_3d.resize(3, imu_sample_num);
  imu_angular_vel_3d.setZero();

  double imu_temp_time = imu_start_time_;
  Eigen::Vector3d imu_single_angular_vel;
  for (int i = 0; i < imu_sample_num; ++i) {
    if (imu_manager_->GetAngleVelocity(imu_temp_time,
                                       &imu_single_angular_vel)) {
      imu_angular_vel_3d.col(i) = imu_single_angular_vel;
    }
    imu_sample_timestamps.push_back(imu_temp_time);
    imu_temp_time += sample_interval;
  }

  // MoCap angular velocity
  mocap_sample_num = static_cast<int>(mocap_duration / sample_interval);
  mocap_sample_timestamps.reserve(mocap_sample_num);
  mocap_angular_vel_3d.resize(3, mocap_sample_num);
  mocap_angular_vel_3d.setZero();

  double mocap_temp_time = mocap_start_time_;
  Eigen::Vector3d mocap_single_angular_vel;
  for (int i = 0; i < mocap_sample_num; ++i) {
    if (mocap_manager_->GetAngleVelocity(mocap_temp_time,
                                         &mocap_single_angular_vel)) {
      mocap_angular_vel_3d.col(i) = mocap_single_angular_vel;
    }
    mocap_sample_timestamps.push_back(mocap_temp_time);
    mocap_temp_time += sample_interval;
  }

  // ===========================================================================

  // Smoothing of the 3D angular velocity.
  Eigen::RowVectorXd smooth_kernel = Eigen::RowVectorXd::Constant(
      smooth_kernal_size, static_cast<double>(1. / smooth_kernal_size));
  Eigen::MatrixXd imu_angular_vel_3d_filtered;
  Eigen::MatrixXd mocap_angular_vel_3d_filtered;
  SignalCorrelation(imu_angular_vel_3d, smooth_kernel,
                    &imu_angular_vel_3d_filtered, true);
  SignalCorrelation(mocap_angular_vel_3d, smooth_kernel,
                    &mocap_angular_vel_3d_filtered, true);

  // ===========================================================================

  // Calculate the norm of angular velocity to obtain the magnitude signal.
  Eigen::RowVectorXd imu_angular_vel_norm(imu_sample_num);
  Eigen::RowVectorXd mocap_angular_vel_norm(mocap_sample_num);
  for (int i = 0; i < imu_angular_vel_3d_filtered.cols(); ++i) {
    imu_angular_vel_norm(i) = imu_angular_vel_3d_filtered.col(i).norm();
  }
  for (int i = 0; i < mocap_angular_vel_3d_filtered.cols(); ++i) {
    mocap_angular_vel_norm(i) = mocap_angular_vel_3d_filtered.col(i).norm();
  }

  // ===========================================================================

  // Calculate the correlation function of the signal and map the maximum index
  // to get the time offset result. Refer to formula (1).
  Eigen::MatrixXd correlation_function(imu_angular_vel_norm.rows(),
                                       imu_sample_num + mocap_sample_num - 1);
  SignalCorrelation(imu_angular_vel_norm, mocap_angular_vel_norm,
                    &correlation_function, false);

  double max_index;
  CorrelationMaxIndex(correlation_function, &max_index);
  *toff_MI_ptr = imu_sample_timestamps.front() -
                 mocap_sample_timestamps.back() + (max_index)*sample_interval;

  // ===========================================================================

  return true;
}

void TimeOffsetInitializer::SignalCorrelation(
    const Eigen::MatrixXd& signal_0, const Eigen::RowVectorXd& signal_1,
    Eigen::MatrixXd* output, const bool& same_size) {
  int rows_0 = signal_0.rows(), rows_output = rows_0, rows_expansion = rows_0;

  constexpr int kSignalMinCols = 2;
  int cols_0 = signal_0.cols(), cols_1 = signal_1.cols();
  if (cols_0 <= kSignalMinCols || cols_1 <= kSignalMinCols) {
    PRINT_ERROR(
        "[Error]: The signal sampling point for calculating correlation must "
        "be greater than %d. \n",
        kSignalMinCols);

    std::exit(EXIT_FAILURE);
  }

  // Determine the dimensions of the output and expansion matrices.
  int cols_output, cols_expansion;
  if (same_size) {
    cols_output = cols_0;
    cols_expansion = cols_0 + cols_1 - 1;
  } else {
    cols_output = cols_0 + cols_1 - 1;
    cols_expansion = cols_0 + 2 * (cols_1 - 1);
  }

  // Expand the signal 0 matrix.
  Eigen::MatrixXd signal_0_expansion(rows_expansion, cols_expansion);
  if (same_size) {
    int left_duplicate_num = (cols_1 - 1) / 2;
    int right_duplicate_num = cols_1 - left_duplicate_num - 1;
    Eigen::MatrixXd expansion_left(rows_0, left_duplicate_num);
    Eigen::MatrixXd expansion_right(rows_0, right_duplicate_num);
    for (int i = 0; i < left_duplicate_num; ++i) {
      expansion_left.col(i) = signal_0.col(0);
    }
    for (int i = 0; i < right_duplicate_num; ++i) {
      expansion_right.col(i) = signal_0.col(cols_0 - 1);
    }
    signal_0_expansion << expansion_left, signal_0, expansion_right;
  } else {
    Eigen::MatrixXd expansion_zeros(rows_0, cols_1 - 1);
    expansion_zeros.setZero();
    signal_0_expansion << expansion_zeros, signal_0, expansion_zeros;
  }

  // Calculate the correlation result.
  output->resize(rows_output, cols_output);
  for (int i = 0; i < rows_output; ++i) {
    for (int j = 0; j < cols_output; ++j) {
      (*output)(i, j) = (signal_0_expansion.row(i).segment(j, cols_1).array() *
                         signal_1.array())
                            .sum();
    }
  }
}

void TimeOffsetInitializer::CorrelationMaxIndex(
    const Eigen::RowVectorXd& correlation_function, double* max_index_ptr,
    const int& fitting_half_range) {
  correlation_function.row(0).maxCoeff(max_index_ptr);

  // Determine the start and end index.
  int fitting_start_index, fitting_end_index;
  fitting_start_index = *max_index_ptr - fitting_half_range;
  fitting_end_index = *max_index_ptr + fitting_half_range;

  if (fitting_start_index < 0 ||
      fitting_end_index > correlation_function.cols()) {
    return;
  }

  // Get the correlation values in range.
  std::vector<double> fitting_values;
  std::vector<double> fitting_index(2 * fitting_half_range + 1);
  for (int i = fitting_start_index; i <= fitting_end_index; ++i) {
    fitting_values.push_back(correlation_function(i));
  }
  std::iota(fitting_index.begin(), fitting_index.end(), 0);

  // Solving for the coefficients of the quadratic polynomial fitting.
  constexpr int kFittingDegree = 2;
  Eigen::MatrixXd design_matrix(fitting_index.size(), kFittingDegree + 1);
  Eigen::VectorXd target_vector(fitting_values.size());
  for (size_t i = 0; i < fitting_index.size(); ++i) {
    target_vector(i) = fitting_values[i];
    for (int j = 0; j <= kFittingDegree; ++j) {
      design_matrix(i, j) = std::pow(fitting_index[i], j);
    }
  }

  // Maximum index = -2*a/b
  Eigen::VectorXd fitting_coeffs =
      design_matrix.householderQr().solve(target_vector);
  *max_index_ptr -= fitting_coeffs[1] / (2 * fitting_coeffs[2]) +
                    fitting_index[fitting_half_range];
}

}  // namespace mocap2gt
