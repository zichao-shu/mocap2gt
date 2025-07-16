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

#include <Eigen/Eigen>
#include <cmath>

namespace mocap2gt {

/**
 * @brief Skew-symmetric matrix from a given 3x1 vector.
 *
 * @param v 3*1 vector to be made a Skew-symmetric.
 * @return 3*3 Skew-symmetric matrix.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(
    const Eigen::MatrixBase<Derived> &r) {
  typedef typename Derived::Scalar scalar;

  Eigen::Matrix<scalar, 3, 3> r_skew = Eigen::Matrix<scalar, 3, 3>::Zero();
  r_skew << static_cast<scalar>(0.), -r[2], r[1], r[2], static_cast<scalar>(0.),
      -r[0], -r[1], r[0], static_cast<scalar>(0.);

  return r_skew;
}

/**
 * @brief Vee operator that extracts a 3x1 vector from a given 3x3
 * Skew-symmetric matrix.
 *
 * @param R 3*3 Skew-symmetric matrix.
 * @return 3*1 vector portion of Skew.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 1> Vee(
    const Eigen::MatrixBase<Derived> &R) {
  typedef typename Derived::Scalar scalar;

  Eigen::Matrix<scalar, 3, 1> r;
  r << R(2, 1), R(0, 2), R(1, 0);
  return r;

  return r;
}

/**
 * @brief Skew-symmetric matrix from a given 6x1 se(3) vector.
 *
 * @param v 6*1 se(3) vector to be made a Skew-symmetric.
 * @return 4*4 Skew-symmetric matrix.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Se3Skew(
    const Eigen::MatrixBase<Derived> &xi) {
  typedef typename Derived::Scalar scalar;

  Eigen::Matrix<scalar, 4, 4> xi_skew = Eigen::Matrix<scalar, 4, 4>::Zero();
  xi_skew.template block<3, 3>(0, 0) = Skew(xi.template block<3, 1>(0, 0));
  xi_skew.template block<3, 1>(0, 3) = xi.template block<3, 1>(3, 0);

  return xi_skew;
}

/**
 * @brief Get right Jacobian of so(3).
 *
 * @param r 3*1 so(3) vector.
 * @return 3*3 left Jacobian of so(3).
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 3> So3Jr(
    const Eigen::MatrixBase<Derived> &r) {
  typedef typename Derived::Scalar scalar;

  scalar theta = r.norm();
  if (theta < static_cast<scalar>(1e-9)) {
    return Eigen::Matrix<scalar, 3, 3>::Identity();
  } else {
    Eigen::Matrix<scalar, 3, 1> a = r / theta;
    Eigen::Matrix<scalar, 3, 3> J =
        std::sin(theta) / theta * Eigen::Matrix<scalar, 3, 3>::Identity() +
        (static_cast<scalar>(1.) - std::sin(theta) / theta) * a *
            a.transpose() -
        ((static_cast<scalar>(1.) - std::cos(theta)) / theta) * Skew(a);

    return J;
  }
}

/**
 * @brief Get right Jacobian's inverse of so(3).
 *
 * @param r 3*1 so(3) vector.
 * @return 3*3 left Jacobian's inverse of so(3).
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 3> So3JrInv(
    const Eigen::MatrixBase<Derived> &r) {
  typedef typename Derived::Scalar scalar;

  scalar theta = r.norm();
  if (theta < static_cast<scalar>(1e-9)) {
    return Eigen::Matrix<scalar, 3, 3>::Identity();
  } else {
    Eigen::Matrix<scalar, 3, 1> a = r / theta;
    Eigen::Matrix<scalar, 3, 3> J =
        (theta / static_cast<scalar>(2.)) /
            std::tan(theta / static_cast<scalar>(2.)) *
            Eigen::Matrix<scalar, 3, 3>::Identity() +
        (static_cast<scalar>(1.) -
         (theta / static_cast<scalar>(2.)) /
             std::tan(theta / static_cast<scalar>(2.))) *
            a * a.transpose() +
        (theta / static_cast<scalar>(2.)) * Skew(a);

    return J;
  }
}

/**
 * @brief so(3) exponential
 *
 * @param r 3*1 so(3) vector.
 * @return 3*3 SO(3) rotation matrix.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 3> So3Exp(
    const Eigen::MatrixBase<Derived> &r) {
  typedef typename Derived::Scalar scalar;

  // Compute with the help of Eigen.
  Eigen::AngleAxis<scalar> aa(r.norm(), r.normalized());
  Eigen::Matrix<scalar, 3, 3> R = aa.toRotationMatrix();

  return R;
}

/**
 * @brief Quaternionic form of the so(3) exponential
 *
 * @param r 3*1 so(3) vector.
 * @return Rotation quaterniond.
 */
template <typename Derived>
static inline Eigen::Quaternion<typename Derived::Scalar> So3ExpQuat(
    const Eigen::MatrixBase<Derived> &r) {
  typedef typename Derived::Scalar scalar;

  // Compute with the help of Eigen.
  Eigen::Quaternion<scalar> q(So3Exp(r));

  return q;
}

/**
 * @brief se(3) exponential
 *
 * @param xi 6*1 se(3) vector.
 * @return 4*4 SE(3) rotation matrix.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Se3Exp(
    const Eigen::MatrixBase<Derived> &xi) {
  typedef typename Derived::Scalar scalar;

  // Compute with the help of Eigen.
  Eigen::Matrix<scalar, 3, 1> w = xi.template block<3, 1>(0, 0);
  Eigen::Matrix<scalar, 3, 1> v = xi.template block<3, 1>(3, 0);
  Eigen::AngleAxis<scalar> aa(w.norm(), w.normalized());
  Eigen::Matrix<scalar, 4, 4> T = Eigen::Matrix<scalar, 4, 4>::Identity();
  T.template block<3, 3>(0, 0) = aa.toRotationMatrix();
  // t = so3Jl(w) * v
  T.template block<3, 1>(0, 3) = So3Jr(-w) * v;

  return T;
}

/**
 * @brief SO(3) logarithm.
 *
 * @param R 3*3 SO(3) rotation matrix.
 * @return 3*1 so(3) vector.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 1> So3Log(
    const Eigen::MatrixBase<Derived> &R) {
  typedef typename Derived::Scalar scalar;

  // Compute with the help of Eigen.
  Eigen::AngleAxis<scalar> aa(R);
  Eigen::Matrix<scalar, 3, 1> r = aa.angle() * aa.axis();

  return r;
}

/**
 * @brief SE(3) logarithm.
 *
 * @param T 4*4 SE(3) transformation matrix.
 * @return 6*1 se(3) vector.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 6, 1> Se3Log(
    const Eigen::MatrixBase<Derived> &T) {
  typedef typename Derived::Scalar scalar;

  // Compute with the help of Eigen.
  Eigen::AngleAxis<scalar> aa(T.template block<3, 3>(0, 0));
  Eigen::Matrix<scalar, 6, 1> xi = Eigen::Matrix<scalar, 6, 1>::Zero();
  Eigen::Matrix<scalar, 3, 1> w = aa.angle() * aa.axis();
  // v = so3JlInv(w) * t
  xi << w, So3JrInv(-w) * T.template block<3, 1>(0, 3);

  return xi;
}

/**
 * @brief Construct rotation matrix from given roll.
 *
 * @param roll Roll angle.
 * @return 3*3 rotation matrix.
 */
template <typename scalar>
static inline Eigen::Matrix<scalar, 3, 3> Roll2Rot(const scalar &roll) {
  Eigen::Matrix<scalar, 3, 3> R = Eigen::Matrix<scalar, 3, 3>::Zero();
  scalar s_roll = std::sin(roll);
  scalar c_roll = std::cos(roll);
  R << static_cast<scalar>(1.), static_cast<scalar>(0.),
      static_cast<scalar>(0.), static_cast<scalar>(0.), c_roll, -s_roll,
      static_cast<scalar>(0.), s_roll, c_roll;

  return R;
}

/**
 * @brief Construct rotation matrix from given pitch.
 *
 * @param pitch Pitch angle.
 * @return 3*3 rotation matrix.
 */
template <typename scalar>
static inline Eigen::Matrix<scalar, 3, 3> Pitch2Rot(const scalar &pitch) {
  Eigen::Matrix<scalar, 3, 3> R = Eigen::Matrix<scalar, 3, 3>::Zero();
  scalar s_pitch = std::sin(pitch);
  scalar c_pitch = std::cos(pitch);
  R << c_pitch, static_cast<scalar>(0.), s_pitch, static_cast<scalar>(0.),
      static_cast<scalar>(1.), static_cast<scalar>(0.), -s_pitch,
      static_cast<scalar>(0.), c_pitch;

  return R;
}

/**
 * @brief Construct rotation matrix from given yaw.
 *
 * @param yaw Yaw angle.
 * @return 3*3 rotation matrix.
 */
template <typename scalar>
static inline Eigen::Matrix<scalar, 3, 3> Yaw2Rot(const scalar &yaw) {
  Eigen::Matrix<scalar, 3, 3> R = Eigen::Matrix<scalar, 3, 3>::Zero();
  scalar s_yaw = std::sin(yaw);
  scalar c_yaw = std::cos(yaw);
  R << c_yaw, -s_yaw, static_cast<scalar>(0.), s_yaw, c_yaw,
      static_cast<scalar>(0.), static_cast<scalar>(0.), static_cast<scalar>(0.),
      static_cast<scalar>(1.);

  return R;
}

/**
 * @brief Gets the quaternion of the input rpy angle (in the order raw, pitch,
 * yaw). Following the external rotation expression: R_output = R_z(yaw) *
 * R_y(pitch) * R_x(roll).
 *
 * @param rpy Input rpy angle.
 * @return Corresponding quaternion.
 */
template <typename Derived>
static inline Eigen::Quaternion<typename Derived::Scalar> Rpy2Quat(
    const Eigen::MatrixBase<Derived> &rpy) {
  typedef typename Derived::Scalar scalar;

  Eigen::Matrix<scalar, 3, 3> roll_R = Roll2Rot(rpy(0));
  Eigen::Matrix<scalar, 3, 3> pitch_R = Pitch2Rot(rpy(1));
  Eigen::Matrix<scalar, 3, 3> yaw_R = Yaw2Rot(rpy(2));

  Eigen::Matrix<scalar, 3, 3> R = yaw_R * pitch_R * roll_R;
  Eigen::Quaternion<scalar> q(R);

  return q;
}

/**
 * @brief Gets the rpy angle (in the order raw, pitch, yaw) of the input
 * quaternion. To recover the external rotation expression: R_input = R_z(yaw) *
 * R_y(pitch) * R_x(roll).
 *
 * @param quat Input quaternion.
 * @return 3*1 rpy values.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 1> Quat2Rpy(
    const Eigen::QuaternionBase<Derived> &quat) {
  typedef typename Derived::Scalar scalar;

  Eigen::Matrix<scalar, 3, 1> rpy;
  Eigen::Matrix<scalar, 3, 3> R = quat.toRotationMatrix();

  rpy(0) = atan2(R(2, 1), R(2, 2));
  rpy(1) = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
  rpy(2) = atan2(R(1, 0), R(0, 0));

  return rpy;
}

/**
 * @brief Ensure the angle is valid and in range [-pi,pi].
 *
 * @param theta Input 1-DoF rotation angle.
 * @return Equivalent rotation within the range [-pi,pi].
 */
template <typename scalar>
static inline scalar Range2Pi(const scalar &theta) {
  scalar target_theta = theta;

  while (target_theta > M_PI) {
    target_theta -= 2 * M_PI;
  }
  while (target_theta < -M_PI) {
    target_theta += 2 * M_PI;
  }

  return target_theta;
}

/**
 * @brief Left product matrix of quaternion.
 * Note that the default order of quaternion here is (qw, qx, qy, qz).
 *
 * @param q Input quaternion.
 * @return 4*4 left product matrix.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 4, 4> QuatLeftProd(
    const Eigen::QuaternionBase<Derived> &q) {
  typedef typename Derived::Scalar scalar;

  Eigen::Matrix<scalar, 4, 4> left_P;
  left_P(0, 0) = q.w();
  left_P.template block<1, 3>(0, 1) = -q.vec().transpose();
  left_P.template block<3, 1>(1, 0) = q.vec();
  left_P.template block<3, 3>(1, 1) =
      q.w() * Eigen::Matrix<scalar, 3, 3>::Identity() + Skew(q.vec());

  return left_P;
}

/**
 * @brief Right product matrix of quaternion.
 * Note that the default order of quaternion here is (qw, qx, qy, qz).
 *
 * @param q Input quaternion.
 * @return 4*4 right product matrix.
 */
template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 4, 4> QuatRightProd(
    const Eigen::QuaternionBase<Derived> &q) {
  typedef typename Derived::Scalar scalar;

  Eigen::Matrix<scalar, 4, 4> right_P;
  right_P(0, 0) = q.w();
  right_P.template block<1, 3>(0, 1) = -q.vec().transpose();
  right_P.template block<3, 1>(1, 0) = q.vec();
  right_P.template block<3, 3>(1, 1) =
      q.w() * Eigen::Matrix<scalar, 3, 3>::Identity() - Skew(q.vec());

  return right_P;
}

/**
 * @brief Rotation angle in degrees for quaternion.
 *
 * @param q Input quaternion.
 * @return Rotation angle in degrees.
 */
template <typename Derived>
static inline double QuatAngleDegree(const Eigen::QuaternionBase<Derived> &q) {
  typedef typename Derived::Scalar scalar;

  Eigen::Quaternion<scalar> q_normalize = q.normalized();
  if (q_normalize.w() < 0) {
    q_normalize.coeffs() = -q_normalize.coeffs();
  }

  scalar angle = 2 * std::acos(q_normalize.w()) * 180. / M_PI;

  return angle;
}

}  // namespace mocap2gt
