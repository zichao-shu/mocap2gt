<div align="center">
  <h1 style="font-size: 1.8em; margin-bottom: 0em">
    <strong>MoCap2GT:</strong>
  </h1>
  <h2 style="font-size: 1.4em; margin-top: 0;">
    A High-Precision <strong>G</strong>round <strong>T</strong>ruth Estimator for SLAM Benchmarking<br>
    Based on <strong>Mo</strong>tion 
    <strong>Cap</strong>ture and IMU Fusion
  </h2>
</div>

<p align="center"><strong>Anonymous Authors</strong></p>
<p align="center">Paper under double-blind review</p>

**MoCap2GT** is an estimator that fuses 6-DoF poses from motion capture (MoCap) with 6-axis IMU measurements to produce **high-precision**, IMU-centric **ground truth** trajectories for rigorous **SLAM benchmarking**. It addresses the insufficient accuracy of MoCap-based ground truth in existing benchmarks (e.g., the well-known EuRoC and TUM-VI datasets), especially in **quantifying rotational and inter-frame relative errors**.

---

## 📚 Table of Contents

- [📚 Table of Contents](#-table-of-contents)
- [✨ Features](#-features)
- [⚙️ Installation](#️-installation)
  - [Manual Dependency Installation and Build](#manual-dependency-installation-and-build)
  - [Docker Setup and Build (Recommended)](#docker-setup-and-build-recommended)
- [🚀 Running the MoCap2GT Estimator](#-running-the-mocap2gt-estimator)
  - [Inputs and Outputs:](#inputs-and-outputs)
- [📊 Running the Data Simulator](#-running-the-data-simulator)
  - [Inputs and Outputs:](#inputs-and-outputs-1)
- [📁 Dataset Access](#-dataset-access)
- [📜 License](#-license)

---

## ✨ Features

- **Spatiotemporal Calibration**: High-precision calibration between the SLAM device's onboard IMU and the MoCap system.

- **MoCap Jitter Mitigation**: Leverages the complementary strengths of IMU and MoCap to suppress high-frequency noise and outliers in the MoCap system, significantly improving ground truth accuracy.

- **Robustness in Real-World Conditions**: Demonstrates robustness to motion degradation and can handle variable time offsets between local clocks.

---

## ⚙️ Installation

### Manual Dependency Installation and Build

This project is primarily developed and tested on Ubuntu 20.04. We recommend using this environment for smooth installation.

1. Install required third-party libraries:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git curl unzip libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev

git clone https://github.com/PX4/eigen.git --branch 3.4 --single-branch
mkdir -p eigen/build && cd eigen/build
cmake ..
make install
cd ../..
rm -rf eigen

git clone https://github.com/jbeder/yaml-cpp.git --branch 0.8.0 --single-branch
mkdir -p yaml-cpp/build && cd yaml-cpp/build
cmake ..
make -j$(nproc)
sudo make install
cd ../..
rm -rf yaml-cpp

git clone https://github.com/ceres-solver/ceres-solver --branch 1.14.0 --single-branch
mkdir -p ceres-solver/build && cd ceres-solver/build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc)
sudo make install
cd ../..
rm -rf ceres-solver

```

2. Build MoCap2GT:

```bash
cd <your-code-repo-dir>/MoCap2GT

mkdir -p build && cd build

cmake .. && make -j$(nproc)

```

Replace `your-code-repo-dir` with the actual path where you cloned or placed the repository.

### Docker Setup and Build (Recommended)

1. Build the Docker image:

```bash
cd <your-code-repo-dir>

docker build -f ./Dockerfile -t run_mocap2gt:v0.1 .
```

2. Run the Docker container interactively with your current directory mounted:

```bash
docker run -it --name test_mocap2gt \
-v $(pwd):/mocap2gt \
run_mocap2gt:v0.1
```

3. Inside the running container, build the project:

```bash
cd /mocap2gt

mkdir -p build && cd build

cmake .. && make -j$(nproc)
```

## 🚀 Running the MoCap2GT Estimator

You can run the MoCap2GT estimator with provided test data by executing:

```bash
cd <your-code-repo-dir>/bin

./GTEstimator ../resource/estimator_test_data/estimator_config.yaml ../resource/estimator_test_data/imu_data.txt ../resource/estimator_test_data/mocap_data.txt ../resource/estimator_test_data/mocap2gt_traj.txt ../resource/estimator_test_data/mocap2gt_calib.yaml
```

### Inputs and Outputs:

- **Inputs:**
  - Estimator configuration (`estimator_config.yaml`)
  - IMU measurements (`imu_data.txt`)
  - Raw MoCap trajectory (`mocap_data.txt`, denoted by T_WM)

- **Outputs:**
  - Optimized ground truth trajectory with respect to the IMU local frame (`mocap2gt_traj.txt`, denoted by T_WI)
  - Calibration parameter results (`mocap2gt_calib.yaml`)

> Note: The configuration file `estimator_config.yaml` contains all the parameters for the estimator. Please modify it according to your dataset and experiment needs.

## 📊 Running the Data Simulator

You can generate synthetic IMU and MoCap data to verify the performance of the estimator by running the data simulator:

```bash
cd <your-code-repo-dir>/bin

./DataSimulator ../resource/simulator_test_data/simulator_config.yaml ../resource/simulator_test_data/input_traj.txt ../resource/simulator_test_data/imu_data.txt ../resource/simulator_test_data/mocap_data.txt
```

### Inputs and Outputs:

- **Inputs:**
  - Simulator configuration (`simulator_config.yaml`)
  - Reference trajectory for simulation (`input_traj.txt`, denoted by T_WI)


- **Outputs:**
  - Simulated IMU measurements (`imu_data.txt`)
  - Simulated MoCap trajectory (`mocap_data.txt`, denoted by T_WM)

> Note: The configuration file `simulator_config.yaml` file contains parameters for simulation; modify it as needed.

After generating simulation data, run the estimator on it:

```bash
./GTEstimator ../resource/simulator_test_data/estimator_config.yaml ../resource/simulator_test_data/imu_data.txt ../resource/simulator_test_data/mocap_data.txt ../resource/simulator_test_data/mocap2gt_traj.txt ../resource/simulator_test_data/mocap2gt_calib.yaml
```

You can then compare the original trajectory (`input_traj.txt`) with the estimated trajectory (`mocap2gt_traj.txt`) to evaluate the estimator’s accuracy, e.g., using [evo](https://github.com/MichaelGrupp/evo):

```bash
evo_ape tum input_traj.txt mocap2gt_traj.txt -a
```

## 📁 Dataset Access

Due to the **double-blind policy** of ongoing submissions, the full benchmark datasets and real-world sequences will be released after paper acceptance through our institution’s official data repository.

---

## 📜 License

Copyright 2025 Anonymous Author(s)

Licensed under the Apache License, Version 2.0 (the "License");  
you may not use this file except in compliance with the License.  
You may obtain a copy of the License at:

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software  
distributed under the License is distributed on an "AS IS" BASIS,  
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  
See the License for the specific language governing permissions and  
limitations under the License.

---

**Stay tuned for upcoming releases featuring additional datasets, improved tools, and extended support for more sensor setups!**

We welcome community feedback and contributions to help advance MoCap2GT.
