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
 * @brief Parameterization of the 6-DoF pose (p_3*1, q_4*1) on the manifold.
 */
class PosePQParameterization : public ceres::LocalParameterization {
 public:
  /**
   * @brief Plus on the manifold.
   *
   * @param x Pointer to the original state.
   * @param delta Plus step in tangent space.
   * @param x_plus_delta Pointer to the updated state.
   */
  bool Plus(const double *x, const double *delta,
            double *x_plus_delta) const override;

  // Linear transformation of the local parameterization.
  bool ComputeJacobian(const double *x, double *jacobian) const override;

  // Global dimension (4+3).
  int GlobalSize() const override { return 7; };

  // Local dimension (3+3).
  int LocalSize() const override { return 6; };
};

}  // namespace mocap2gt
