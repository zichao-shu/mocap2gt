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

#pragma once

#include <memory>
#include <vector>

#include "mocap2gt/ceres_factor/imu_preintegration_factor.h"
#include "mocap2gt/ceres_factor/mocap_interpolation_factor.h"
#include "mocap2gt/ceres_factor/mocap_interpolation_variable_factor.h"
#include "mocap2gt/sensor_meas/imu_meas_manager.h"
#include "mocap2gt/utils/config_parser.h"

namespace mocap2gt {

/**
 * @brief Class for estimating ground-truth trajectory by tightly coupling IMU
 * and MoCap measurements.
 */
class MoCap2GTEstimator {
 public:
  // Type definition of aligned timestamped IMU state vector.
  typedef std::vector<Eigen::Matrix<double, 17, 1>,
                      Eigen::aligned_allocator<Eigen::Matrix<double, 17, 1>>>
      TimestampStateVec;

  /**
   * @brief Construct the MoCap2GT estimator, pass the pointers and configure
   * parameters.
   *
   * @param c_parser Config Parser.
   * @param i_manager IMU measurement manager.
   * @param m_manager MoCap measurement manager.
   * @param t_timestamp Timestamps of the ground-truth trajectory to be
   * estimated.
   */
  MoCap2GTEstimator(const std::shared_ptr<ConfigParser> c_parser,
                    const std::shared_ptr<ImuMeasManager> i_manager,
                    const std::shared_ptr<MoCapMeasManagerBase> m_manager);

  /**
   * @brief Build and initialize the ceres problem, and optimize with
   * relinearization iterations.
   */
  void BuildAndOptimize();

  /**
   * @brief Read the estimated IMU states from memory.
   *
   * @param time_states_ptr 17-dimensional IMU state (timestamp, p_MI, q_MI,
   * v_MI, ba, bg).
   */
  void GetImuStates(TimestampStateVec *time_states_ptr);

  /**
   * @brief Read the estimated calibration parameters from memory.
   *
   * @param toff_ptr Time offset, (toff_MI).
   * @param p_MI_ptr Extrinsic translation part, (p_MI).
   * @param q_MI_ptr Extrinsic rotation part, (q_MI).
   * @param q_WG_ptr Gravity alignment rotation, (q_WG).
   */
  void GetCalibParameters(std::vector<double> *toff_ptr,
                          Eigen::Vector3d *p_MI_ptr,
                          Eigen::Quaterniond *q_MI_ptr,
                          Eigen::Quaterniond *q_WG_ptr);

 private:
  // Set the initial guesses of the calibration parameters.
  void InitializeCalibParameters();

  // Set the initial guesses of the imu states.
  void InitialImuStates();

  /**
   * @brief Build the ceres problem during relinearization iterations.
   *
   * @param first_relin First relinearization flag.
   */
  void BuildProblem(bool first_relin);

  /**
   * @brief Solve the ceres problem during relinearization iterations.
   * @return Ceres solution usable
   */
  bool OptimizeProblem();

  // Ceres problem.
  ceres::Problem problem_;

  // IMU preintegration residual block historical vector, for updating the
  // residual block during relinearization iterations.
  std::vector<ceres::ResidualBlockId> imu_preinteg_blocks_;

  // Config parser to get the config parameters.
  std::shared_ptr<ConfigParser> config_parser_;
  // IMU manager used to obtain IMU measurements and Jacobian.
  std::shared_ptr<ImuMeasManager> imu_manager_;
  // MoCap manager used to obtain MoCap measurements and Jacobian.
  std::shared_ptr<MoCapMeasManagerBase> mocap_manager_;

  // Holding the pointers to the memory of the states to be optimized.
  // n*7 IMU poses at timestamp i: p_WI(x, y, z), q_WI(x, y, z, w).
  std::shared_ptr<double[]> para_pose_WI_;
  // n*9 IMU speeds and biases at timestamp i: v_WI(x, y, z), ba(x, y, z)，bg(x,
  // y, z).
  std::shared_ptr<double[]> para_speed_bias_WI_;
  // 1*2 2-DoF gravity alignment rotation: rot_WG(roll, pitch).
  std::shared_ptr<double[]> para_rotxy_WG_;
  // 1*7 Extrinsic between IMU and MoCap: p_MI(x, y, z), q_MI(x, y, z, w).
  std::shared_ptr<double[]> para_extrinsic_MI_;
  // 1*n Variable time offsets between IMU and MoCap: toff_MI, (IMU time = MoCap
  // time + toff_MI).
  std::shared_ptr<double[]> para_toff_MI_;

  // Timestamps of control points for variable time offsets under the IMU clock.
  std::vector<double> toff_control_points;
  // Truncate time offset to avoid large states.
  double toff_truncate_;

  // Timestamps of the ground-truth trajectory to be estimated.
  std::vector<double> target_timestamps_;
  // Number of poses (timestamps) in the ground-truth trajectory to be
  // estimated.
  int est_traj_pose_num_;

  // Count of relinearization of IMU bias in the estimation.
  int relin_iter_count_;
  // Flag for whether the state is to be optimized.
  bool est_T_MI_flag_;
  bool est_q_WG_flag_;
  bool est_toff_MI_flag_;
};

}  // namespace mocap2gt
