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

#include <iomanip>

#include "mocap2gt/ceres_estimator/mocap2gt_estimator.h"
#include "mocap2gt/utils/data_reader.h"

int main(int argc, char **argv) {
  mocap2gt::Printer::SetPrintLevel(mocap2gt::Printer::PrintLevel::INFO);

  // Step 1: Parse the arguments.
  if (argc != 6) {
    // clang-format off
    PRINT_ERROR(
      "[Input Error]: Invalid input arguments.\n\n"
      "Expected usage:\n"
      "  ./GTEstimator <estimator_config.yaml> <imu_data.txt> <mocap_data.txt> <output_traj.txt> <output_calib.yaml>\n\n"
      "Arguments:\n"
      "  1) estimator_config.yaml  : Path to the estimator configuration\n"
      "  2) imu_data.txt           : Path to the input IMU measurement\n"
      "  3) mocap_data.txt         : Path to the input MoCap trajectory (T_WM)\n"
      "  4) output_traj.txt        : Path to the output estimated trajectory (T_WI)\n"
      "  5) output_calib.yaml      : Path to the output calibration parameter\n"
    );
    // clang-format on

    std::exit(EXIT_FAILURE);
  }

  std::string config_file_path = argv[1];
  std::string imu_data_path = argv[2];
  std::string mocap_data_path = argv[3];
  std::string output_traj_path = argv[4];
  std::string output_calib_path = argv[5];

  // ===========================================================================

  // Step 2: Parse the config and initialize the manager.
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

  // ===========================================================================

  // Step 3: Read the sensor data, build and solve the problem.
  PRINT_INFO(
      "===================== Build and optimize. =====================\n");
  // Read the data and feed them to the manager.
  mocap2gt::DataReader::ReadImuData(imu_meas_manager, imu_data_path);
  mocap2gt::DataReader::ReadMoCapData(mocap_meas_manager, mocap_data_path);

  // If modeling with B-spline, we need to generate control points first.
  if (config_parser->get_use_spline_model_flag()) {
    mocap_meas_manager->GenerateControlPoints();
  }

  // Initialize the estimator.
  std::shared_ptr<mocap2gt::MoCap2GTEstimator> estimator(
      new mocap2gt::MoCap2GTEstimator(config_parser, imu_meas_manager,
                                      mocap_meas_manager));

  // Build and optimize the ceres problem.
  estimator->BuildAndOptimize();

  // ===========================================================================

  // Step 4: Save the estimated states, and view the results of the calibration
  // parameters.
  PRINT_INFO(
      "====================== View the results. ======================\n");
  mocap2gt::MoCap2GTEstimator::TimestampStateVec time_states;
  estimator->GetImuStates(&time_states);

  Eigen::Vector3d p_MI_initial = config_parser->get_p_MI_init();
  Eigen::Quaterniond q_MI_initial = config_parser->get_q_MI_init();
  Eigen::Quaterniond q_WG_initial = config_parser->get_q_WG_init();
  double toff_initial = config_parser->get_toff_MI_init();

  Eigen::Vector3d p_MI_result;
  Eigen::Quaterniond q_MI_result;
  Eigen::Quaterniond q_WG_result;
  std::vector<double> toff_result;
  estimator->GetCalibParameters(&toff_result, &p_MI_result, &q_MI_result,
                                &q_WG_result);

  if (time_states.size() == 0) {
    PRINT_ERROR(
        "[Error]: The IMU states is empty, make sure you build the problem. "
        "\n");

    std::exit(EXIT_FAILURE);
  }

  // Step 4.1: Save the estimated GT trajectory (T_WI).
  std::ofstream output_traj_file(output_traj_path);
  if (output_traj_file.is_open()) {
    output_traj_file << std::fixed << std::setprecision(9);
    for (size_t i = 0; i < time_states.size(); ++i) {
      Eigen::Matrix<double, 17, 1> time_state = time_states[i];
      for (int j = 0; j < 8; j++) {
        output_traj_file << time_state(j);
        if (j < 7) {
          output_traj_file << " ";
        }
      }
      output_traj_file << std::endl;
    }
  } else {
    PRINT_ERROR("[Error]: Failed to open output trajectory file: %s \n",
                output_traj_path.c_str());
    std::exit(EXIT_FAILURE);
  }
  output_traj_file.close();
  PRINT_INFO("[Info]: The result trajectory (T_WI) has been saved to: %s \n",
             output_traj_path.c_str());

  // Step 4.2: Save the estimated calibration parameters.
  std::ofstream output_calib_file(output_calib_path);
  if (output_calib_file.is_open()) {
    output_calib_file << std::fixed << std::setprecision(9);
    output_calib_file << "p_MI: [" << p_MI_result.x() << ", " << p_MI_result.y()
                      << ", " << p_MI_result.z() << "]" << std::endl;
    output_calib_file << "q_MI: [" << q_MI_result.x() << ", " << q_MI_result.y()
                      << ", " << q_MI_result.z() << ", " << q_MI_result.w()
                      << "]" << std::endl;
    output_calib_file << "q_WG: [" << q_WG_result.x() << ", " << q_WG_result.y()
                      << ", " << q_WG_result.z() << ", " << q_WG_result.w()
                      << "]" << std::endl;
    // For variable time offset we only save the first control point.
    output_calib_file << "toff_MI: " << toff_result[0] << std::endl;
  } else {
    PRINT_ERROR(
        "[Error]: Failed to open output calibration parameters file: %s \n",
        output_calib_path.c_str());
    std::exit(EXIT_FAILURE);
  }
  output_calib_file.close();
  PRINT_INFO(
      "[Info]: The result calibration parameters have been saved to: %s \n",
      output_calib_path.c_str());

  // Step 4.3: View the results of the spatiotemporal calibration parameters.
  PRINT_INFO(
      "[Info]: Extrinsic p_MI: (%.6f, %.6f, %.6f) -> (%.6f, %.6f, %.6f) \n",
      p_MI_initial.x(), p_MI_initial.y(), p_MI_initial.z(), p_MI_result.x(),
      p_MI_result.y(), p_MI_result.z());
  PRINT_INFO(
      "[Info]: Extrinsic q_MI: (%.6f, %.6f, %.6f, %.6f) -> (%.6f, %.6f, %.6f, "
      "%.6f) \n",
      q_MI_initial.x(), q_MI_initial.y(), q_MI_initial.z(), q_MI_initial.w(),
      q_MI_result.x(), q_MI_result.y(), q_MI_result.z(), q_MI_result.w());
  PRINT_INFO(
      "[Info]: Gravity alignment q_WG: (%.6f, %.6f, %.6f, %.6f) -> (%.6f, "
      "%.6f, %.6f, %.6f) \n",
      q_WG_initial.x(), q_WG_initial.y(), q_WG_initial.z(), q_WG_initial.w(),
      q_WG_result.x(), q_WG_result.y(), q_WG_result.z(), q_WG_result.w());
  // For variable time offset we only print the first control point.
  PRINT_INFO("[Info]: Time offset toff_MI: %.9f -> %.9f \n", toff_initial,
             toff_result[0]);
  for (size_t i = 0; i < toff_result.size(); ++i) {
    PRINT_INFO("and %.9f \n", toff_result[i]);
  }

  // ===========================================================================

  return 0;
}
