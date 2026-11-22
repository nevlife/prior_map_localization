#include "lio_twist.hpp"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <cmath>

namespace prior_map_localization
{
namespace
{

Eigen::Matrix3d yaw(double rad)
{
  return Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

TEST(BaseTwist, StraightMotionIsInThePreviousImuFrame)
{
  const ImuMount mount{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
  const LioPose prev{yaw(M_PI / 2), Eigen::Vector3d(0.0, 0.0, 0.0)};
  const LioPose cur{yaw(M_PI / 2), Eigen::Vector3d(0.0, 0.2, 0.0)};
  const BaseTwist t = base_twist(mount, prev, cur, 0.1);
  EXPECT_NEAR(t.linear.x(), 2.0, 1e-12);
  EXPECT_NEAR(t.linear.y(), 0.0, 1e-12);
  EXPECT_NEAR(t.angular.norm(), 0.0, 1e-12);
}

// The base turns in place, so the IMU 1 m ahead of it sweeps sideways while the base stands still.
// The chord of the sweep leaves a second-order residue in x.
TEST(BaseTwist, LeverArmRemovesTheImuSweep)
{
  const ImuMount mount{Eigen::Matrix3d::Identity(), Eigen::Vector3d(-1.0, 0.0, 0.0)};
  const double wz = 0.5;
  const double dt = 0.1;
  const LioPose prev{yaw(0.0), Eigen::Vector3d(1.0, 0.0, 0.0)};
  const LioPose cur{yaw(wz * dt), Eigen::Vector3d(std::cos(wz * dt), std::sin(wz * dt), 0.0)};
  const BaseTwist t = base_twist(mount, prev, cur, dt);
  EXPECT_NEAR(t.angular.z(), wz, 1e-12);
  EXPECT_NEAR(t.linear.x(), 0.0, 0.02);
  EXPECT_NEAR(t.linear.y(), 0.0, 1e-3);
}

TEST(BaseTwist, MountRotationTurnsTheRatesIntoTheBaseFrame)
{
  // IMU mounted upside down about x: its z axis points down.
  const Eigen::Matrix3d flip = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const ImuMount mount{flip, Eigen::Vector3d::Zero()};
  const LioPose prev{yaw(0.0), Eigen::Vector3d::Zero()};
  const LioPose cur{yaw(0.05), Eigen::Vector3d(0.1, 0.0, 0.0)};
  const BaseTwist t = base_twist(mount, prev, cur, 0.1);
  EXPECT_NEAR(t.angular.z(), -0.5, 1e-12);
  EXPECT_NEAR(t.linear.x(), 1.0, 1e-12);
}

// The vectornav mount: right-forward-up axes, 0.129 m ahead of and 0.714 m above the base.
ImuMount vectornav_mount()
{
  return {yaw(-M_PI / 2), Eigen::Vector3d(-0.129, 0.0, -0.714)};
}

TEST(StateTwist, WorldVelocityBecomesForwardBaseVelocity)
{
  // The robot heads along world y, so IMU x (right) points along world x.
  const ImuMount mount = vectornav_mount();
  const Eigen::Matrix3d imu_in_world = yaw(M_PI / 2) * mount.base_rotation;
  const BaseTwist t =
    base_twist(mount, imu_in_world, Eigen::Vector3d(0.0, 1.5, 0.0), Eigen::Vector3d::Zero());
  EXPECT_NEAR(t.linear.x(), 1.5, 1e-12);
  EXPECT_NEAR(t.linear.y(), 0.0, 1e-12);
}

TEST(StateTwist, LeverArmRemovesTheImuSweep)
{
  const ImuMount mount = vectornav_mount();
  const double wz = 0.5;
  // Turning in place, the IMU 0.129 m ahead of the base moves left at wz * 0.129.
  const Eigen::Matrix3d imu_in_world = mount.base_rotation;
  const Eigen::Vector3d imu_velocity(0.0, wz * 0.129, 0.0);
  const BaseTwist t = base_twist(mount, imu_in_world, imu_velocity, Eigen::Vector3d(0.0, 0.0, wz));
  EXPECT_NEAR(t.angular.z(), wz, 1e-12);
  EXPECT_NEAR(t.linear.x(), 0.0, 1e-12);
  EXPECT_NEAR(t.linear.y(), 0.0, 1e-12);
}

TEST(StateTwist, CovarianceFollowsTheHeading)
{
  const Eigen::Matrix3d world = Eigen::Vector3d(4.0, 1.0, 0.0).asDiagonal();
  const ImuMount mount = vectornav_mount();
  const Eigen::Matrix3d imu_in_world = yaw(M_PI / 2) * mount.base_rotation;
  const Eigen::Matrix3d base = base_covariance(mount, imu_in_world, world);
  EXPECT_NEAR(base(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(base(1, 1), 4.0, 1e-12);
  EXPECT_NEAR(base(0, 1), 0.0, 1e-12);
}

TEST(ImuBias, IsTheBaseYawBiasInTheImuFrame)
{
  const Eigen::Matrix3d flip = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d bias = imu_bias({flip, Eigen::Vector3d::Zero()}, 0.01);
  EXPECT_NEAR(bias.x(), 0.0, 1e-12);
  EXPECT_NEAR(bias.y(), 0.0, 1e-12);
  EXPECT_NEAR(bias.z(), -0.01, 1e-12);
}

}  // namespace
}  // namespace prior_map_localization
