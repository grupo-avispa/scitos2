// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
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

#ifndef SCITOS2_MODULES__ODOMETRY_COVARIANCE_HPP_
#define SCITOS2_MODULES__ODOMETRY_COVARIANCE_HPP_

#include <algorithm>
#include <cmath>
#include <mutex>

#include "nav_msgs/msg/odometry.hpp"

namespace scitos2_modules
{

/**
 * @class scitos2_modules::OdometryCovariance
 * @brief Covariances of the odometry from the noise coefficients of the robot model.
 *
 * The model of the robot (`getRobotModel`) carries the four coefficients of the odometry
 * motion model of Thrun, Burgard and Fox (*Probabilistic Robotics*), as standard deviations per
 * metre or radian travelled. The pose covariance is the variance accumulated along the path
 * since the last reset, as dead reckoning grows without bound. The twist covariance follows the
 * current velocity. Both have a floor so that they are never zero, which would make a filter
 * such as `robot_localization` trust the odometry blindly.
 */
class OdometryCovariance
{
public:
  /**
   * @struct Noise
   * @brief Noise coefficients of the odometry motion model of the robot.
   */
  struct Noise
  {
    /** @brief Translation error per metre travelled. */
    double trans_trans{0.0};

    /** @brief Rotation error per metre travelled. */
    double trans_rot{0.0};

    /** @brief Translation error per radian turned. */
    double rot_trans{0.0};

    /** @brief Rotation error per radian turned. */
    double rot_rot{0.0};
  };

  /**
   * @brief Set the noise coefficients.
   *
   * @param noise Coefficients of the robot model
   */
  void setNoise(const Noise & noise)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    noise_ = noise;
  }

  /**
   * @brief Forget the accumulated variance and the last pose, as after resetting the odometry.
   */
  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_last_pose_ = false;
    translation_variance_ = 0.0;
    rotation_variance_ = 0.0;
  }

  /**
   * @brief Accumulate the variance of the motion since the last pose and fill the covariances
   * of an odometry message.
   *
   * @param odometry Message with the pose and the velocity already set
   */
  void update(nav_msgs::msg::Odometry & odometry)
  {
    const double x = odometry.pose.pose.position.x;
    const double y = odometry.pose.pose.position.y;
    const auto & q = odometry.pose.pose.orientation;
    const double phi = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double linear = odometry.twist.twist.linear.x;
    const double angular = odometry.twist.twist.angular.z;

    std::lock_guard<std::mutex> lock(mutex_);
    if (has_last_pose_) {
      const double distance = std::hypot(x - last_x_, y - last_y_);
      const double turn = std::fabs(std::remainder(phi - last_phi_, 2.0 * M_PI));
      const double translation_std = noise_.trans_trans * distance + noise_.rot_trans * turn;
      const double rotation_std = noise_.trans_rot * distance + noise_.rot_rot * turn;
      translation_variance_ += translation_std * translation_std;
      rotation_variance_ += rotation_std * rotation_std;
    }
    last_x_ = x;
    last_y_ = y;
    last_phi_ = phi;
    has_last_pose_ = true;

    // Row-major 6x6 matrices over (x, y, z, roll, pitch, yaw): the pose is planar, so the
    // axes that cannot be observed are given a large variance
    odometry.pose.covariance.fill(0.0);
    odometry.twist.covariance.fill(0.0);
    const double position_variance = translation_variance_ + kMinVariance;
    const double yaw_variance = rotation_variance_ + kMinVariance;
    odometry.pose.covariance[0] = position_variance;
    odometry.pose.covariance[7] = position_variance;
    odometry.pose.covariance[14] = kUnobservedVariance;
    odometry.pose.covariance[21] = kUnobservedVariance;
    odometry.pose.covariance[28] = kUnobservedVariance;
    odometry.pose.covariance[35] = yaw_variance;

    const double linear_std = noise_.trans_trans * std::fabs(linear) +
      noise_.rot_trans * std::fabs(angular);
    const double angular_std = noise_.trans_rot * std::fabs(linear) +
      noise_.rot_rot * std::fabs(angular);
    odometry.twist.covariance[0] = linear_std * linear_std + kMinVariance;
    odometry.twist.covariance[7] = kMinVariance;
    odometry.twist.covariance[14] = kUnobservedVariance;
    odometry.twist.covariance[21] = kUnobservedVariance;
    odometry.twist.covariance[28] = kUnobservedVariance;
    odometry.twist.covariance[35] = angular_std * angular_std + kMinVariance;
  }

private:
  /** @brief Smallest variance reported, in m^2 or rad^2. */
  static constexpr double kMinVariance{1e-4};

  /** @brief Variance of the axes that a planar robot cannot measure. */
  static constexpr double kUnobservedVariance{1e6};

  /** @brief Protects all the members. */
  std::mutex mutex_;

  /** @brief Noise coefficients of the robot model. */
  Noise noise_;

  /** @brief Whether last_x_, last_y_ and last_phi_ hold a pose. */
  bool has_last_pose_{false};

  /** @brief X of the last pose, in m. */
  double last_x_{0.0};

  /** @brief Y of the last pose, in m. */
  double last_y_{0.0};

  /** @brief Yaw of the last pose, in rad. */
  double last_phi_{0.0};

  /** @brief Variance accumulated in x and y, in m^2. */
  double translation_variance_{0.0};

  /** @brief Variance accumulated in yaw, in rad^2. */
  double rotation_variance_{0.0};
};

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__ODOMETRY_COVARIANCE_HPP_
