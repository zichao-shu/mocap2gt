FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

# RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g' /etc/apt/sources.list

# Basic dependencies. 
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    unzip \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev

# Install Eigen3 3.3.7 from source code. 
RUN git clone https://github.com/PX4/eigen.git --branch 3.4 --single-branch && \
    mkdir -p eigen/build && \
    cd eigen/build && \
    cmake .. && \
    make install && \
    cd ../.. && \
    rm -rf eigen

# Install yaml-cpp 0.8.0 from source code. 
RUN git clone https://github.com/jbeder/yaml-cpp.git --branch 0.8.0 --single-branch && \
    mkdir -p yaml-cpp/build && \
    cd yaml-cpp/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf yaml-cpp

# Install Ceres solver 1.14.0 from source code. 
RUN git clone https://github.com/ceres-solver/ceres-solver --branch 1.14.0 --single-branch && \
    mkdir -p ceres-solver/build && \
    cd ceres-solver/build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf ceres-solver

RUN apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*
