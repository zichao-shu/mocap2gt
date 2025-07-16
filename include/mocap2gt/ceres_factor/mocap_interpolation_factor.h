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

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <memory>

#include "mocap2gt/sensor_meas/mocap_meas_manager_linear.h"
#include "mocap2gt/sensor_meas/mocap_meas_manager_spline.h"
#include "mocap2gt/utils/printer.h"

namespace mocap2gt {

/**
 * @brief MoCap interpolation factor. We compute the 6-dimensional residual and
 * find its Jacobian with respect to each state. The states involved include:
 * - 7*1 IMU pose at time i (T_WI_i).
 * - 7*1 spatial hand-eye calibration parameter (T_MI).
 * - 1*1 temporal hand-eye calibration parameter (toff_MI).
 */
class MoCapInterpolationFactor : public ceres::SizedCostFunction<6, 7, 7, 1> {
 public:
  /**
   * @brief Construct the MoCap interpolation factor.
   *
   * @param manager MoCap measurement manager.
   * @param t_I_i Timestamp of the MoCap interpolation factor under the IMU
   * clock.
   * @param toff_truncate Time offset truncation, to avoid unstable
   * optimization.
   */
  MoCapInterpolationFactor(std::shared_ptr<MoCapMeasManagerBase> manager,
                           double t_I_i, double toff_truncate = 0.)
      : mocap_manager_(manager), t_I_i_(t_I_i), toff_truncate_(toff_truncate) {}

  /**
   * @brief Compute residual and Jacobians based on the involved states.
   *
   * @param parameters The current value of involved states.
   * @param residuals Output 6-dimensional residual.
   * @param jacobians Output Jacobians with respect to each state.
   */
  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override;

 private:
  // MoCap manager used to obtain MoCap measurements and Jacobian.
  std::shared_ptr<MoCapMeasManagerBase> mocap_manager_;

  // Timestamp of T_WI_i.
  double t_I_i_;
  // Time offset truncation.
  double toff_truncate_;
};

}  // namespace mocap2gt
