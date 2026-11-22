#ifndef LIO_TWIST_HPP_
#define LIO_TWIST_HPP_

#include <Eigen/Core>

namespace prior_map_localization
{

// The IMU mount from the base->imu transform: its rotation, and the lever arm from the IMU to
// the base origin in the base frame.
struct ImuMount
{
  Eigen::Matrix3d base_rotation;
  Eigen::Vector3d imu_to_base;
};

struct LioPose
{
  Eigen::Matrix3d rotation;
  Eigen::Vector3d position;
};

struct BaseTwist
{
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

// The mean base twist between two FAST-LIO IMU poses dt_sec apart.
BaseTwist base_twist(
  const ImuMount & mount, const LioPose & prev, const LioPose & cur, double dt_sec);

// The base twist from the IMU rotation and velocity in the world and its body angular velocity.
BaseTwist base_twist(
  const ImuMount & mount, const Eigen::Matrix3d & rotation, const Eigen::Vector3d & velocity,
  const Eigen::Vector3d & angular_velocity);

// A world-frame velocity covariance in the base frame.
Eigen::Matrix3d base_covariance(
  const ImuMount & mount, const Eigen::Matrix3d & rotation, const Eigen::Matrix3d & covariance);

// A yaw-rate bias about the base z axis, in the IMU frame.
Eigen::Vector3d imu_bias(const ImuMount & mount, double yaw_bias);

}  // namespace prior_map_localization

#endif  // LIO_TWIST_HPP_
