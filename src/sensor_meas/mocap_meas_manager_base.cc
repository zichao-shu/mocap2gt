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

#include "mocap2gt/sensor_meas/mocap_meas_manager_base.h"

namespace mocap2gt {

bool MoCapMeasManagerBase::FeedPose(const double &timestamp,
                                    const Eigen::Vector3d &p,
                                    const Eigen::Quaterniond &q) {
  // Assert the timestamps are positive and in ascending order.
  if (timestamp < 0 || timestamp <= end_time_) {
    PRINT_ERROR(
        "[Error]: Feed MoCap data error, timestamp is negative or does not "
        "increase: %f",
        timestamp);

    return false;
  }

  if (start_time_ < 0) {
    start_time_ = timestamp;
  }

  if (end_time_ > 0 && std::abs(timestamp - end_time_) >= 0.1) {
    PRINT_WARNING(
        "[Warning]: Missing MoCap data for more than 0.1s after %f may lead to "
        "inaccurate trajectory estimation.. \n",
        timestamp);
  }
  end_time_ = timestamp;

  // Convert the pose into SE(3) matrix.
  Eigen::Matrix4d T_WM = Eigen::Matrix4d::Identity();
  T_WM.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T_WM.block<3, 1>(0, 3) = p;

  traj_points_map_.insert({timestamp, T_WM});

  return true;
}

}  // namespace mocap2gt
