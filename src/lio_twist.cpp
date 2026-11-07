#include "lio_twist.hpp"

#include <Eigen/Geometry>

namespace prior_map_localization
{

BaseTwist base_twist(
  const ImuMount & mount, const LioPose & prev, const LioPose & cur, double dt_sec)
{
  const Eigen::Vector3d v_imu = prev.rotation.transpose() * (cur.position - prev.position) / dt_sec;
  const Eigen::AngleAxisd turn(prev.rotation.transpose() * cur.rotation);
  const Eigen::Vector3d w = mount.base_rotation * (turn.angle() * turn.axis() / dt_sec);
  const Eigen::Vector3d v = mount.base_rotation * v_imu + w.cross(mount.imu_to_base);
  return {v, w};
}

Eigen::Vector3d imu_bias(const ImuMount & mount, double yaw_bias)
{
  return mount.base_rotation.transpose() * Eigen::Vector3d(0, 0, yaw_bias);
}

}  // namespace prior_map_localization
