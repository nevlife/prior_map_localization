#include "pose_tracker.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace prior_map_localization
{
namespace
{

Eigen::Matrix4f at(float x, float y, float yaw)
{
  Eigen::Affine3f pose(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
  pose.translation() = Eigen::Vector3f(x, y, 0.0f);
  return pose.matrix();
}

TwistSample twist(double stamp, float vx, float wz)
{
  return {stamp, Eigen::Vector3f(vx, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, wz)};
}

TEST(PoseTracker, SeedMovesAlongTheBodyAxes)
{
  PoseTracker tracker(0.5);
  tracker.reset(at(1.0f, 2.0f, static_cast<float>(M_PI / 2)), 10.0);
  tracker.add_twist(twist(10.0, 2.0f, 0.4f));
  const Eigen::Matrix4f seed = tracker.seed(10.25);
  EXPECT_NEAR(seed(0, 3), 1.0f, 1e-5f);
  EXPECT_NEAR(seed(1, 3), 2.5f, 1e-5f);
  EXPECT_NEAR(std::atan2(seed(1, 0), seed(0, 0)), M_PI / 2 + 0.1, 1e-5);
}

TEST(PoseTracker, SeedCapsThePredictionInterval)
{
  PoseTracker tracker(0.5);
  tracker.reset(at(0.0f, 0.0f, 0.0f), 10.0);
  tracker.add_twist(twist(10.0, 2.0f, 0.0f));
  EXPECT_NEAR(tracker.seed(13.0)(0, 3), 1.0f, 1e-6f);
}

TEST(PoseTracker, SeedStaysWithoutAnEarlierTwist)
{
  PoseTracker tracker(0.5);
  tracker.reset(at(3.0f, 0.0f, 0.0f), 10.0);
  tracker.add_twist(twist(10.5, 2.0f, 0.0f));
  EXPECT_TRUE(tracker.seed(10.2).isApprox(at(3.0f, 0.0f, 0.0f)));
  EXPECT_TRUE(tracker.seed(9.0).isApprox(at(3.0f, 0.0f, 0.0f)));
}

TEST(PoseTracker, MotionNeedsTwoAcceptedScansInARow)
{
  PoseTracker tracker(0.5);
  tracker.reset(at(0.0f, 0.0f, 0.0f), 10.0);
  tracker.accept(at(1.0f, 0.0f, 0.0f), 10.1);
  EXPECT_FALSE(tracker.motion().has_value());
  tracker.accept(at(2.0f, 0.0f, 0.1f), 10.2);
  ASSERT_TRUE(tracker.motion().has_value());
  EXPECT_NEAR(tracker.motion()->translation.x(), 1.0f, 1e-5f);
  EXPECT_NEAR(tracker.motion()->duration_sec, 0.1, 1e-9);
  EXPECT_NEAR(
    tracker.motion()->rotation.angularDistance(Eigen::Quaternionf::Identity()), 0.1f, 1e-5f);
}

TEST(PoseTracker, RejectMovesThePredictionAndBreaksTheRun)
{
  PoseTracker tracker(0.5);
  tracker.reset(at(0.0f, 0.0f, 0.0f), 10.0);
  tracker.accept(at(1.0f, 0.0f, 0.0f), 10.1);
  tracker.reject(at(1.5f, 0.0f, 0.0f), 10.2);
  tracker.reject(at(1.8f, 0.0f, 0.0f), 10.3);
  EXPECT_EQ(tracker.consecutive_rejects(), 2);
  EXPECT_DOUBLE_EQ(tracker.accepted_stamp_sec(), 10.1);
  EXPECT_TRUE(tracker.seed(10.3).isApprox(at(1.8f, 0.0f, 0.0f)));
  tracker.accept(at(2.0f, 0.0f, 0.0f), 10.4);
  EXPECT_EQ(tracker.consecutive_rejects(), 0);
  EXPECT_FALSE(tracker.motion().has_value());
}

TEST(PoseCovariance, GrowsWithFitnessUpToTheCap)
{
  EXPECT_DOUBLE_EQ(capped_std(0.2, 0.1, 5.0, -1.0), 0.2);
  EXPECT_DOUBLE_EQ(capped_std(0.2, 0.1, 5.0, 2.0), 0.4);
  EXPECT_DOUBLE_EQ(capped_std(0.2, 0.1, 5.0, 100.0), 5.0);
  const PoseStd pose_std{{0.2, 0.3, 0.02, 0.03}, {0.1, 0.1, 0.01, 0.01}, {5.0, 5.0, 0.5, 0.5}};
  const std::array<double, 36> c = pose_covariance(pose_std, 1.0);
  EXPECT_NEAR(c[0], 0.09, 1e-12);
  EXPECT_NEAR(c[7], 0.09, 1e-12);
  EXPECT_NEAR(c[14], 0.16, 1e-12);
  EXPECT_NEAR(c[21], 0.0009, 1e-12);
  EXPECT_NEAR(c[28], 0.0009, 1e-12);
  EXPECT_NEAR(c[35], 0.0016, 1e-12);
  EXPECT_DOUBLE_EQ(c[1], 0.0);
}

}  // namespace
}  // namespace prior_map_localization
