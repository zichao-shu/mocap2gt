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

#include "mocap2gt/sensor_meas/imu_meas_manager.h"
#include "mocap2gt/sensor_meas/mocap_meas_manager_linear.h"
#include "mocap2gt/sensor_meas/mocap_meas_manager_spline.h"

namespace mocap2gt {

/**
 * @brief Extrinsic solver elements, each containing MoCap and IMU
 * preintegration observations for a certain time period, are used to construct
 * the linear solver system.
 */
struct ExtrinsicSolverElement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct a extrinsic solver element.
   *
   * @param integrator The IMU preintegration result over a period of time.
   * @param p_0 Translational part of the MaCap measurement at the start time.
   * @param q_0 Rotational part of the MaCap measurement at the start time.
   * @param p_1 Translational part of the MaCap measurement at the end time.
   * @param q_1 Rotational part of the MaCap measurement at the end time.
   * @param delta_t Length of the time period.
   * @param high_quality Flag indicating if it is a high-quality element.
   */
  ExtrinsicSolverElement(const std::shared_ptr<ImuPreintegrator>& integrator,
                         const Eigen::Vector3d& p_0,
                         const Eigen::Quaterniond& q_0,
                         const Eigen::Vector3d& p_1,
                         const Eigen::Quaterniond& q_1, const double& delta_t,
                         const bool& high_quality = false)
      : preintegrator(integrator),
        p_W_Mi(p_0),
        q_W_Mi(q_0),
        p_W_Mj(p_1),
        q_W_Mj(q_1),
        delta_time(delta_t),
        high_quality_flag(high_quality) {}

  // IMU preintegration result.
  std::shared_ptr<ImuPreintegrator> preintegrator;
  // MaCap measurement.
  Eigen::Vector3d p_W_Mi;
  Eigen::Quaterniond q_W_Mi;
  Eigen::Vector3d p_W_Mj;
  Eigen::Quaterniond q_W_Mj;
  // Length of the time period.
  double delta_time;
  // Flag for high-quality element.
  bool high_quality_flag;
};

/**
 * @brief Classes for estimating the initial guesses of spatial extrinsic
 * between IMU and MoCap (p_MI, q_MI) and gravity aligned rotation (q_WG).
 * We perform linear least squares optimization based on the relationship
 * constraints between IMU preintegration and the relative pose from motion
 * capture.
 */
class ExtrinsicInitializer {
 public:
  /**
   * @brief Construct the extrinsic initializer, pass the pointers and
   * initialize the start and end times.
   *
   * @param i_manager IMU measurement manager.
   * @param m_manager MoCap measurement manager.
   * @param truncate_time Truncate time of the data.
   */
  ExtrinsicInitializer(const std::shared_ptr<ImuMeasManager> i_manager,
                       const std::shared_ptr<MoCapMeasManagerBase> m_manager,
                       const double& cut_off_time, const double& toff_MI);

  /**
   * @brief Interface function to estimate the initial guess of the extrinsic.
   *
   * @param p_MI_ptr Translational extrinsic between IMU and MoCap (p_MI).
   * @param q_MI_ptr Rotational extrinsic between IMU and MoCap (q_MI).
   * @param q_WG_ptr Gravity aligned rotation (q_WG).
   * @param gravity_magnitude Gravity magnitude.
   * @param max_sample_interval Maximum sample interval of the solver element.
   * @param element_trans_thresh Relative rotation threshold for the solver
   * element.
   * @param element_rot_thresh Relative translation threshold for the solver
   * element.
   * @param max_sample_num Maximum number of samples to control the scale of the
   * solver.
   * @return True if able to initialize the extrinsic.
   */
  bool EstimateExtrinsic(Eigen::Vector3d* p_MI_ptr,
                         Eigen::Quaterniond* q_MI_ptr,
                         Eigen::Quaterniond* q_WG_ptr,
                         const double& gravity_magnitude,
                         const double& max_sample_interval = 0.1,
                         const double& element_trans_thresh = 0.2,
                         const double& element_rot_thresh = 5,
                         const int& max_sample_num = 500);

 private:
  /**
   * @brief Estimating the initial guess of the rotational extrinsic (q_MI).
   *
   * @param solver_elements Solver elements for constructing the linear solving
   * system.
   * @param q_MI_ptr Initial guess of the rotational extrinsic.
   * @param robust_coeff Amplification factor of the robust kernel.
   */
  void EstimateRotExtrinsic(
      const std::vector<ExtrinsicSolverElement>& solver_elements,
      Eigen::Quaterniond* q_MI_ptr, const int& robust_coeff = 5);

  /**
   * @brief Simultaneously estimate the initial values of the translational
   * extrinsic (p_MI) and gravity alignment (q_WG).
   *
   * @param solver_elements Solver elements for constructing the linear solving
   * system.
   * @param p_MI The obtained rotational extrinsic.
   * @param p_MI_ptr Initial guess of the translational extrinsic.
   * @param gravity_ptr Gravity represented in W.
   */
  void EstimateTransExtrinsicGravityAlign(
      const std::vector<ExtrinsicSolverElement>& solver_elements,
      const Eigen::Quaterniond& q_MI, Eigen::Vector3d* p_MI_ptr,
      Eigen::Vector3d* gravity_ptr);

  /**
   * @brief Iteratively refine the initial guesses using the constraint of
   * gravity magnitude.
   *
   * @param solver_elements Solver elements for constructing the linear solving
   * system.
   * @param q_MI The obtained rotational extrinsic.
   * @param p_MI_ptr Initial guess of the translational extrinsic.
   * @param gravity_ptr Gravity represented in W.
   * @param gravity_magnitude Gravity magnitude
   * @param iter_num The number of iterations.
   */
  void RefineGravityAlign(
      const std::vector<ExtrinsicSolverElement>& solver_elements,
      const Eigen::Quaterniond& q_MI, Eigen::Vector3d* p_MI_ptr,
      Eigen::Vector3d* gravity_ptr, const double& gravity_magnitude,
      const int& iter_num = 4);

  /**
   * @brief Obtain the tangent space basis of the gravity vector.
   *
   * @param gravity Current gravity vector represented in W.
   * @return 3*2 Tangent space basis.
   */
  Eigen::Matrix<double, 3, 2> GravityTangentBasis(
      const Eigen::Vector3d& gravity);

  // IMU manager used to obtain IMU measurements.
  std::shared_ptr<ImuMeasManager> imu_manager_;
  // MoCap manager used to obtain MoCap measurements.
  std::shared_ptr<MoCapMeasManagerBase> mocap_manager_;

  // Start/end times of measurements.
  double imu_start_time_;
  double imu_end_time_;
  double mocap_start_time_;
  double mocap_end_time_;
  // Time offset between IMU and MoCap.
  double toff_MI_;
};

}  // namespace mocap2gt
