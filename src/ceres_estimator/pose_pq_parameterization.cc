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

#include "mocap2gt/ceres_estimator/pose_pq_parameterization.h"

#include "mocap2gt/utils/geometry_utils.h"

namespace mocap2gt {

bool PosePQParameterization::Plus(const double *x, const double *delta,
                                  double *x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> p(x);
  Eigen::Map<const Eigen::Quaterniond> q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);
  const Eigen::Quaterniond dq(
      So3Exp(Eigen::Map<const Eigen::Vector3d>(delta + 3)));

  Eigen::Map<Eigen::Vector3d> p_plus(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta + 3);

  p_plus = p + dp;
  q_plus = (q * dq).normalized();

  return true;
}

bool PosePQParameterization::ComputeJacobian(const double *x,
                                             double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
  J.topRows<6>().setIdentity();
  J.bottomRows<1>().setZero();

  return true;
}

}  // namespace mocap2gt
