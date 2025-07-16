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
#include <iomanip>
#include <memory>
#include <random>

#include "mocap2gt/sensor_meas/mocap_meas_manager_spline.h"
#include "mocap2gt/utils/simulator_param.h"

/**
 * @brief Add noise to IMU data, and add gravity measurement.
 *
 * @param param Noise parameter.
 * @param q_WI Current IMU rotation.
 * @param acc_ptr Accelerometer measurement we will add noise to.
 * @param gyr_ptr Gyroscope measurement we will add noise to.
 * @param ba_ptr Accelerometer bias we will add noise to.
 * @param bg_ptr Gyroscope bias we will add noise to.
 */
void addIMUNoise(const std::shared_ptr<const mocap2gt::SimulatorParam> param,
                 const Eigen::Quaterniond &q_WI, Eigen::Vector3d *acc_ptr,
                 Eigen::Vector3d *gyr_ptr, Eigen::Vector3d *ba_ptr,
                 Eigen::Vector3d *bg_ptr) {
  // Convert noise std from continuous time to discrete time.
  const double imu_freq = param->get_imu_freq();
  const Eigen::Vector4d imu_noise = param->get_imu_noise();
  const double acc_noise = imu_noise[0] * sqrt(imu_freq);
  const double acc_b_noise = imu_noise[1] / sqrt(imu_freq);
  const double gyr_noise = imu_noise[2] * sqrt(imu_freq);
  const double gyr_b_noise = imu_noise[3] / sqrt(imu_freq);

  // Random seed
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist(0.0, 1.0);
  // Noise to be added.
  const Eigen::Vector3d acc_noise_v =
      acc_noise * Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
  const Eigen::Vector3d acc_b_noise_v =
      acc_b_noise * Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
  const Eigen::Vector3d gyr_noise_v =
      gyr_noise * Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
  const Eigen::Vector3d gyr_b_noise_v =
      gyr_b_noise * Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
  const Eigen::Vector3d gravity_in_G =
      Eigen::Vector3d(0., 0., param->get_gravity_magnitude());
  // Gravity misalignment.
  const Eigen::Quaterniond q_WG = param->get_q_WG();

  // Add noise to IMU measurement, and add gravity to acceleration.
  *acc_ptr +=
      q_WI.inverse() * q_WG * gravity_in_G + *ba_ptr + acc_noise * acc_noise_v;
  *gyr_ptr += *bg_ptr + gyr_noise_v;
  // Add noise to bias.
  *ba_ptr += acc_b_noise_v;
  *bg_ptr += gyr_b_noise_v;
}

/**
 * @brief Add noise to MoCap data, and transform with extrinsic.
 *
 * @param param Noise parameter.
 * @param start_time Start time of sensor data.
 * @param timestamp_ptr Data timestamp we will add noise to.
 * @param p_WI_ptr MoCap translational measurement we will add noise to.
 * @param q_WI_ptr MoCap rotational measurement we will add noise to.
 * @param toff_drift Time offset drift we will add noise to.
 */
void addMoCapNoise(const std::shared_ptr<const mocap2gt::SimulatorParam> param,
                   double *timestamp_ptr, Eigen::Vector3d *p_WI_ptr,
                   Eigen::Quaterniond *q_WI_ptr, double *toff_drift) {
  // Random seed
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist(0.0, 1.0);

  // Time offset noise to be added. Convert noise std from continuous time to
  // discrete time.
  const double mocap_freq = param->get_mocap_freq();
  const double toff_drift_noise =
      param->get_toff_drift_noise() / mocap_freq / 1000. / 60.;

  // MoCap measurement noise to be added.
  const Eigen::Vector3d trans_noise_v = (dist(gen) *
                                        param->get_mocap_trans_noise().x()) *
                                        Eigen::Vector3d::Random().normalized();
  const Eigen::Vector3d rot_noise_v = (dist(gen) *
                                      param->get_mocap_rot_noise().x()) *
                                      Eigen::Vector3d::Random().normalized();
  // Extrinsic. (transform from IMU to MoCap)
  const Eigen::Vector3d p_MI = param->get_p_MI();
  const Eigen::Quaterniond q_MI = param->get_q_MI();

  // Add noise to timestamp.
  *toff_drift += toff_drift_noise;
  *timestamp_ptr = *timestamp_ptr - (param->get_toff_MI() + *toff_drift);

  // Add noise to MoCap measurement and transform with extrinsic.
  *p_WI_ptr = *p_WI_ptr - *q_WI_ptr * q_MI.inverse() * p_MI + trans_noise_v;
  *q_WI_ptr = *q_WI_ptr * q_MI.inverse() * mocap2gt::So3ExpQuat(rot_noise_v);
}

int main(int argc, char **argv) {
  mocap2gt::Printer::SetPrintLevel(mocap2gt::Printer::INFO);

  // Step 1: Parse the arguments.
  if (argc != 5) {
    // clang-format off
    PRINT_ERROR(
      "[Input Error]: Invalid input arguments.\n\n"
      "Expected usage:\n"
      "  ./DataSimulator <simulator_config.yaml> <input_traj.txt> <output_imu.txt> <output_mocap.txt>\n\n"
      "Arguments:\n"
      "  1) simulator_config.yaml  : Path to the simulator configuration\n"
      "  2) input_traj.txt         : Path to the input reference trajectory for simulation (T_WI)\n"
      "  3) output_imu.txt         : Path to the output simulated IMU measurement\n"
      "  4) output_mocap.txt       : Path to the output simulated MoCap trajectory (T_WM)\n"
    );
    // clang-format on
  
    std::exit(EXIT_FAILURE);
  }

  std::string config_file_path = argv[1];
  std::string input_traj_path = argv[2];
  std::string output_imu_path = argv[3];
  std::string output_mocap_path = argv[4];

  // ===========================================================================

  // Step 2: Parse the config, initialize the SE(3) spline and feed the poses.
  PRINT_INFO(
      "=============== Parse the config of simulator. =============== \n");
  std::shared_ptr<mocap2gt::SimulatorParam> simulator_param(
      new mocap2gt::SimulatorParam(config_file_path));

  // We reuse the SE(3) B-spline modeling in MoCap measurement manager.
  std::shared_ptr<mocap2gt::MoCapMeasManagerSpline> se3_cubic_spline(
      new mocap2gt::MoCapMeasManagerSpline(Eigen::Vector3d::Zero(),
                                           Eigen::Vector3d::Zero()));

  std::ifstream input_traj_file(input_traj_path);
  if (!input_traj_file) {
    PRINT_ERROR("[Error]: Failed to open input trajectory file: %s \n",
                input_traj_path.c_str());

    std::exit(EXIT_FAILURE);
  }

  // Read the known input trajectory (T_MI) and feed the B-spline.
  std::string line;
  while (std::getline(input_traj_file, line)) {
    std::istringstream ss(line);
    std::string value;
    std::vector<double> values;

    while (std::getline(ss, value, ' ')) {
      values.push_back(std::stod(value));
    }

    if (values.size() != 8) {
      PRINT_ERROR(
          "[Error]: Timestamped poses in the input trajectory file must be in "
          "TUM format, i.e. "
          "\"timestamp(s) tx(m) ty(m) tz(m) qx qy qz qw\" \n")
      std::exit(EXIT_FAILURE);
    }

    double timestamp(values[0]);
    Eigen::Vector3d p_WI(values[1], values[2], values[3]);
    Eigen::Quaterniond q_WI(values[7], values[4], values[5], values[6]);

    se3_cubic_spline->FeedPose(timestamp, p_WI, q_WI);
  }

  // ===========================================================================

  // Step 3: Obtain the target timestamps, simulate the data through B-splines,
  // add the configured noise and output.
  PRINT_INFO(
      "=========== Generate simulated MoCap and IMU data. =========== \n");
  se3_cubic_spline->GenerateControlPoints();

  const double spline_start_time = se3_cubic_spline->get_start_time();
  const double spline_end_time = se3_cubic_spline->get_end_time();
  const double mocap_freq = simulator_param->get_mocap_freq();
  const double imu_freq = simulator_param->get_imu_freq();
  std::vector<double> mocap_timestamps;
  std::vector<double> imu_timestamps;
  mocap_timestamps.reserve(
      static_cast<int>((spline_end_time - spline_start_time) * mocap_freq) + 1);
  imu_timestamps.reserve(
      static_cast<int>((spline_end_time - spline_start_time) * imu_freq) + 1);

  // MoCap timestamps.
  double current_time = spline_start_time;
  while (current_time < spline_end_time) {
    mocap_timestamps.push_back(current_time);
    current_time += (1. / mocap_freq);
  }

  // IMU timestamps.
  current_time = spline_start_time;
  while (current_time < spline_end_time) {
    imu_timestamps.push_back(current_time);
    current_time += (1. / imu_freq);
  }

  // Output file stream.
  std::ofstream output_mocap_file(output_mocap_path);
  std::ofstream output_imu_file(output_imu_path);
  output_mocap_file << std::fixed << std::setprecision(9);
  output_imu_file << std::fixed << std::setprecision(9);

  // Step 3.1: Simulate IMU data (acc, gyr in local frame I) and add the noise.
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  for (auto timestamp : imu_timestamps) {
    Eigen::Vector3d p_WI;
    Eigen::Quaterniond q_WI;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;

    // Get the noise-free measurement.
    if (!se3_cubic_spline->GetPoseInertialData(timestamp, &p_WI, &q_WI, &acc,
                                               &gyr)) {
      continue;
    }

    // Add configured IMU noise and the gravity measurement.
    addIMUNoise(simulator_param, q_WI, &acc, &gyr, &ba, &bg);
    output_imu_file << static_cast<int64_t>(timestamp * 1e9) << ", " << gyr.x()
                    << ", " << gyr.y() << ", " << gyr.z() << ", " << acc.x()
                    << ", " << acc.y() << ", " << acc.z() << std::endl;
  }

  // Step 3.2: Simulate MoCap data (T_WM) and add the noise.
  double toff_drift = 0.;
  for (auto timestamp : mocap_timestamps) {
    Eigen::Vector3d p_WM;
    Eigen::Quaterniond q_WM;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;

    // Get the noise-free measurement.
    if (!se3_cubic_spline->GetPoseInertialData(timestamp, &p_WM, &q_WM, &acc,
                                               &gyr)) {
      continue;
    }

    // Add configured MoCap noise.
    addMoCapNoise(simulator_param, &timestamp, &p_WM, &q_WM, &toff_drift);
    output_mocap_file << timestamp << " " << p_WM.x() << " " << p_WM.y() << " "
                      << p_WM.z() << " " << q_WM.x() << " " << q_WM.y() << " "
                      << q_WM.z() << " " << q_WM.w() << std::endl;
  }

  input_traj_file.close();
  output_mocap_file.close();
  output_imu_file.close();

  PRINT_INFO("[Info]: Data generation done. \n");

  // ===========================================================================

  return 0;
}
