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

namespace mocap2gt {

/**
 * @brief Parameterization of the 2-DoF gravity-aligned rotation (roll pitch).
 */
class RotRPParameterization : public ceres::LocalParameterization {
  /**
   * @brief Update and limit the range of angles
   *
   * @param x Pointer to the original state.
   * @param delta Plus step.
   * @param x_plus_delta Pointer to the updated state.
   */
  bool Plus(const double *x, const double *delta,
            double *x_plus_delta) const override;

  // Linear transformation of the local parameterization.
  bool ComputeJacobian(const double *x, double *jacobian) const override;

  // Global dimension (1+1).
  int GlobalSize() const override { return 2; }

  // Local dimension (1+1).
  int LocalSize() const override { return 2; }
};

}  // namespace mocap2gt
