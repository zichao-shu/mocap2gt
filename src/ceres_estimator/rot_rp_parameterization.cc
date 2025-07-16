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

#include "mocap2gt/ceres_estimator/rot_rp_parameterization.h"

#include "mocap2gt/utils/geometry_utils.h"

namespace mocap2gt {

bool RotRPParameterization::Plus(const double *x, const double *delta,
                                 double *x_plus_delta) const {
  // Ensure the angle is in range [-pi, pi]
  x_plus_delta[0] = Range2Pi(x[0] + Range2Pi(delta[0]));
  x_plus_delta[1] = Range2Pi(x[1] + Range2Pi(delta[1]));

  return true;
}

bool RotRPParameterization::ComputeJacobian(const double *x,
                                            double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J(jacobian);
  J.setIdentity();

  return true;
}

}  // namespace mocap2gt
