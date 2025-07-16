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

#include <fstream>

#include "mocap2gt/initializer/extrinsic_initializer.h"
#include "mocap2gt/initializer/time_offset_initializer.h"
#include "mocap2gt/utils/config_parser.h"
#include "mocap2gt/utils/data_reader.h"

int main(int argc, char **argv) {
  mocap2gt::Printer::SetPrintLevel(mocap2gt::Printer::PrintLevel::DEBUG);

  // Parse the arguments.
  std::string config_file_path = argv[1];
  std::string imu_data_path = argv[2];
  std::string mocap_data_path = argv[3];

  // ===========================================================================

  // Parse the config and initialize the manager.
  PRINT_INFO(
      "================ Parse the config of estimator. ===============\n");
  // Initialize the config parser.
  std::shared_ptr<mocap2gt::ConfigParser> config_parser(
      new mocap2gt::ConfigParser(config_file_path));
  // Initialize the IMU measurement manager.
  std::shared_ptr<mocap2gt::ImuMeasManager> imu_meas_manager(
      new mocap2gt::ImuMeasManager(config_parser->get_imu_freq(),
                                   config_parser->get_imu_noise(),
                                   config_parser->get_gravity_magnitude()));
  // Initialize the MoCap measurement manager, model based on linear or cubic
  // B-spline interpolation.
  std::shared_ptr<mocap2gt::MoCapMeasManagerBase> mocap_meas_manager;
  if (config_parser->get_use_spline_model_flag()) {
    mocap_meas_manager.reset(new mocap2gt::MoCapMeasManagerSpline(
        config_parser->get_mocap_trans_noise(),
        config_parser->get_mocap_rot_noise()));
  } else {
    mocap_meas_manager.reset(new mocap2gt::MoCapMeasManagerLinear(
        config_parser->get_mocap_trans_noise(),
        config_parser->get_mocap_rot_noise()));
  }

  // Read the data and feed them to the manager.
  mocap2gt::DataReader::ReadImuData(imu_meas_manager, imu_data_path);
  mocap2gt::DataReader::ReadMoCapData(mocap_meas_manager, mocap_data_path);

  // If modeling with B-spline, we need to generate control points first.
  if (config_parser->get_use_spline_model_flag()) {
    mocap_meas_manager->GenerateControlPoints();
  }

  // ===========================================================================

  // Test the initializer and compare the results with the ground-truth.
  PRINT_DEBUG(
      "==================== Test the initializer. ====================\n");
  // Test the time offset initializer.
  double toff_MI_init;
  std::shared_ptr<mocap2gt::TimeOffsetInitializer> toff_initializer(
      new mocap2gt::TimeOffsetInitializer(imu_meas_manager, mocap_meas_manager,
                                          config_parser->get_max_toff()));
  toff_initializer->EstimateToff(&toff_MI_init);

  // Compare the result of toff_MI.
  double toff_MI_gt = config_parser->get_toff_MI_init();
  double toff_MI_error = toff_MI_init - toff_MI_gt;
  PRINT_DEBUG("[Debug]: Initialization of toff_MI: \n")
  PRINT_DEBUG("[Debug]: gt: %.6f s, init: %.6f s, error: %.6f s.\n", toff_MI_gt,
              toff_MI_init, toff_MI_error);

  // ===========================================================================

  // Test the extrinsic initializer.
  Eigen::Vector3d p_MI_init;
  Eigen::Quaterniond q_MI_init;
  Eigen::Quaterniond q_WG_init;
  std::shared_ptr<mocap2gt::ExtrinsicInitializer> ex_initializer(
      new mocap2gt::ExtrinsicInitializer(imu_meas_manager, mocap_meas_manager,
                                         config_parser->get_max_toff(),
                                         toff_MI_init));
  ex_initializer->EstimateExtrinsic(&p_MI_init, &q_MI_init, &q_WG_init,
                                    config_parser->get_gravity_magnitude());

  // Compare the result of p_MI.
  Eigen::Vector3d p_MI_gt = config_parser->get_p_MI_init();
  double p_MI_error = (p_MI_init - p_MI_gt).norm();
  PRINT_DEBUG("[Debug]: Initialization of p_MI: \n")
  PRINT_DEBUG(
      "[Debug]: gt: (%.6f, %.6f, %.6f) init: (%.6f, %.6f, %.6f), error: %.6f "
      "m.\n",
      p_MI_gt.x(), p_MI_gt.y(), p_MI_gt.z(), p_MI_init.x(), p_MI_init.y(),
      p_MI_init.z(), p_MI_error);

  // Compare the result of q_MI.
  Eigen::Quaterniond q_MI_gt = config_parser->get_q_MI_init();
  double q_MI_error =
      Eigen::AngleAxisd(q_MI_init.inverse() * q_MI_gt).angle() * 180. / M_PI;
  PRINT_DEBUG("[Debug]: Initialization of q_MI: \n")
  PRINT_DEBUG(
      "[Debug]: gt: (%.6f, %.6f, %.6f, %.6f) init: (%.6f, %.6f, %.6f, %.6f), "
      "error: %.6f deg.\n",
      q_MI_gt.x(), q_MI_gt.y(), q_MI_gt.z(), q_MI_gt.w(), q_MI_init.x(),
      q_MI_init.y(), q_MI_init.z(), q_MI_init.w(), q_MI_error);

  // Compare the result of q_WG.
  Eigen::Quaterniond q_WG_gt = config_parser->get_q_WG_init();
  Eigen::Vector3d gravity_gt = q_WG_gt * Eigen::Vector3d::UnitZ();
  Eigen::Vector3d gravity_init = q_WG_init * Eigen::Vector3d::UnitZ();
  double q_WG_error = std::acos(gravity_gt.dot(gravity_init) /
                                (gravity_gt.norm() * gravity_init.norm())) *
                      180.0 / M_PI;
  PRINT_DEBUG("[Debug]: Initialization of q_WG: \n")
  PRINT_DEBUG(
      "[Debug]: gt: (%.6f, %.6f, %.6f, %.6f) init: (%.6f, %.6f, %.6f, %.6f), "
      "error: %.6f deg.\n",
      q_WG_gt.x(), q_WG_gt.y(), q_WG_gt.z(), q_WG_gt.w(), q_WG_init.x(),
      q_WG_init.y(), q_WG_init.z(), q_WG_init.w(), q_WG_error);

  // ===========================================================================

  return 0;
}