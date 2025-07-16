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

#include "mocap2gt/sensor_meas/imu_preintegrator.h"
#include "mocap2gt/utils/printer.h"

namespace mocap2gt {

/**
 * @brief IMU preintegration factor. We compute the 15-dimensional residual and
 * find its Jacobians with respect to each state. The states involved include:
 * - 7*1 IMU pose at time i (T_WI_i).
 * - 9*1 IMU speed and bias at time i (v_WI_i & bias_i).
 * - 7*1 IMU pose at time j(T_WI_j).
 * - 9*1 IMU speed and bias at time j (v_WI_i & bias_j).
 * - 2*1 Gravity aligned rotation (rot_WG).
 */
class IMUPreintegrationFactor
    : public ceres::SizedCostFunction<15, 7, 9, 7, 9, 2> {
 public:
  /**
   * @brief Construct the IMU preintegration factor.
   *
   * @param integrator IMU preintegrator to get integration information.
   */
  explicit IMUPreintegrationFactor(std::shared_ptr<ImuPreintegrator> integrator)
      : preintegrator_(integrator) {}

  /**
   * @brief Compute residual and Jacobians based on the involved states.
   *
   * @param parameters The current value of involved states.
   * @param residuals Output 15-dimensional residual.
   * @param jacobians Output Jacobians with respect to each state.
   */
  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override;

 private:
  // IMU preintegrator to obtain IMU measurements and Jacobian.
  std::shared_ptr<ImuPreintegrator> preintegrator_;
};

}  // namespace mocap2gt
