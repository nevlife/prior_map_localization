#include "bbs_search.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace prior_map_localization
{
namespace
{

constexpr double deg = M_PI / 180.0;

BbsParams params()
{
  return {0.5, 1.0 * deg, 6, 2.0, 50.0, 300};
}

TEST(Pose2, InverseUndoesCompose)
{
  const Pose2 a{3.0, -2.0, 30.0 * deg};
  const Pose2 b{1.0, 4.0, -100.0 * deg};
  const Pose2 back = compose(compose(a, b), inverse(b));
  EXPECT_NEAR(back.x, a.x, 1e-12);
  EXPECT_NEAR(back.y, a.y, 1e-12);
  EXPECT_NEAR(back.yaw, a.yaw, 1e-12);
}

TEST(Pose2, ComposeWrapsYaw)
{
  const Pose2 p = compose({0.0, 0.0, 170.0 * deg}, {1.0, 0.0, 20.0 * deg});
  EXPECT_NEAR(p.yaw, -170.0 * deg, 1e-12);
  EXPECT_NEAR(p.x, std::cos(170.0 * deg), 1e-12);
  EXPECT_NEAR(p.y, std::sin(170.0 * deg), 1e-12);
}

TEST(SearchRadius, GrowsWithSigmaAndAgeWithinLimits)
{
  const GpsRadiusParams gps{3.0, 10.0, 30.0, 3.0};
  EXPECT_DOUBLE_EQ(search_radius(gps, 1.0, 1.0, 10.0, 10.0), 10.0);
  EXPECT_DOUBLE_EQ(search_radius(gps, 4.0, 9.0, 10.0, 8.0), 3.0 * 3.0 + 3.0 * 2.0);
  EXPECT_DOUBLE_EQ(search_radius(gps, 100.0, 1.0, 10.0, 10.0), 30.0);
}

TEST(ScanXy, KeepsOnePointPerCellInRange)
{
  const std::vector<Eigen::Vector3f> corners{
    {1.0f, 0.0f, 0.0f},
    {10.1f, 0.1f, 0.0f},
    {10.2f, 0.2f, 0.0f},
    {20.0f, 0.0f, 0.0f},
    {60.0f, 0.0f, 0.0f}};
  Eigen::Affine3f base_from_sensor = Eigen::Affine3f::Identity();
  base_from_sensor.translation() = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  const auto scan = scan_xy(corners, base_from_sensor, params());
  ASSERT_EQ(scan.size(), 3u);
  EXPECT_FLOAT_EQ(scan[0][0], 2.0f);
  EXPECT_FLOAT_EQ(scan[1][0], 11.1f);
  EXPECT_FLOAT_EQ(scan[2][0], 21.0f);
}

TEST(ScanXy, ThinsEvenlyToTheCap)
{
  std::vector<Eigen::Vector3f> corners;
  for (int i = 0; i < 10; ++i) {
    corners.emplace_back(3.0f + i, 0.0f, 0.0f);
  }
  BbsParams p = params();
  p.max_scan_points = 4;
  const auto scan = scan_xy(corners, Eigen::Affine3f::Identity(), p);
  ASSERT_EQ(scan.size(), 4u);
  EXPECT_FLOAT_EQ(scan[0][0], 3.0f);
  EXPECT_FLOAT_EQ(scan[1][0], 5.0f);
  EXPECT_FLOAT_EQ(scan[2][0], 8.0f);
  EXPECT_FLOAT_EQ(scan[3][0], 10.0f);
}

// Three walls of a 20 m square: the corner layer and, seen from the robot, the scan.
std::vector<Eigen::Vector3f> walls()
{
  std::vector<Eigen::Vector3f> out;
  for (float t = 0.0f; t <= 20.0f; t += 0.25f) {
    out.emplace_back(t, 0.0f, 0.0f);
    out.emplace_back(0.0f, t, 0.0f);
    out.emplace_back(t, 20.0f, 0.0f);
  }
  return out;
}

// The walls with a corner at (-0.125, -0.125), which sets the grid origin there, so that no wall
// point falls on a cell boundary.
std::vector<Eigen::Vector3f> corner_layer()
{
  std::vector<Eigen::Vector3f> out = walls();
  out.emplace_back(-0.125f, -0.125f, 0.0f);
  return out;
}

TEST(CornerGrid, CropIsEmptyOffTheMap)
{
  const CornerGrid grid(walls(), 0.5);
  EXPECT_EQ(grid.width(), 41);
  EXPECT_EQ(grid.height(), 41);
  EXPECT_FALSE(grid.crop(500.0, 500.0, 10.0, 50.0).has_value());
  const std::optional<GridCrop> crop = grid.crop(5.0, 5.0, 10.0, 50.0);
  ASSERT_TRUE(crop.has_value());
  EXPECT_EQ(crop->x0, 0);
  EXPECT_EQ(crop->grid.width, 41);
}

TEST(CornerGrid, SearchFindsTheRobotPose)
{
  const CornerGrid grid(corner_layer(), 0.5);
  const Pose2 map_from_base{6.125, 4.125, 30.0 * deg};
  const Pose2 base_from_map = inverse(map_from_base);
  std::vector<std::array<double, 2>> scan;
  for (const Eigen::Vector3f & p : walls()) {
    const Pose2 q = compose(base_from_map, {p.x(), p.y(), 0.0});
    scan.push_back({q.x, q.y});
  }
  const std::optional<GridCrop> crop = grid.crop(5.0, 5.0, 5.0, 30.0);
  ASSERT_TRUE(crop.has_value());
  // Depth 6 blocks (32 m) are wider than the robot's distance to the map edge.
  BbsParams p = params();
  p.pyramid_depth = 6;
  const std::optional<BbsMatch> match = grid.search(*crop, scan, p);
  ASSERT_TRUE(match.has_value());
  EXPECT_NEAR(match->map_from_base.x, map_from_base.x, 1e-9);
  EXPECT_NEAR(match->map_from_base.y, map_from_base.y, 1e-9);
  EXPECT_NEAR(match->map_from_base.yaw, map_from_base.yaw, 1e-9);
  EXPECT_DOUBLE_EQ(match->score, 1.0);
  EXPECT_LT(match->runner_up, match->score);
}

PoseVote vote()
{
  return PoseVote({3, 1.0, 5.0 * deg, 4.0});
}

TEST(PoseVote, NeedsEnoughAgreeingEstimates)
{
  PoseVote v = vote();
  EXPECT_FALSE(v.add({10.0, 5.0, 0.0}, 0.0));
  EXPECT_FALSE(v.add({30.0, 5.0, 0.0}, 1.0));
  EXPECT_FALSE(v.add({10.5, 5.0, 3.0 * deg}, 2.0));
  EXPECT_TRUE(v.add({10.2, 5.2, -1.0 * deg}, 3.0));
}

TEST(PoseVote, ForgetsOldEstimates)
{
  PoseVote v = vote();
  EXPECT_FALSE(v.add({10.0, 5.0, 0.0}, 0.0));
  EXPECT_FALSE(v.add({10.0, 5.0, 0.0}, 1.0));
  EXPECT_FALSE(v.add({10.0, 5.0, 0.0}, 4.5));
  v.clear();
  EXPECT_FALSE(v.add({10.0, 5.0, 0.0}, 5.0));
}

TEST(PoseVote, ComparesYawAcrossTheWrap)
{
  PoseVote v = vote();
  v.add({0.0, 0.0, 179.0 * deg}, 0.0);
  v.add({0.0, 0.0, -179.0 * deg}, 0.1);
  EXPECT_TRUE(v.add({0.0, 0.0, 180.0 * deg}, 0.2));
}

TEST(GroundHeight, TakesTheFifthPercentileNearby)
{
  std::vector<Eigen::Vector3f> points;
  for (int i = 0; i < 40; ++i) {
    points.emplace_back(0.0f, 0.0f, static_cast<float>(i));
  }
  points.emplace_back(10.0f, 0.0f, -50.0f);
  const GroundHeight ground(points, 2.0);
  EXPECT_DOUBLE_EQ(*ground.at(0.5, 0.5), 2.0);
  EXPECT_FALSE(ground.at(5.0, 5.0).has_value());
}

}  // namespace
}  // namespace prior_map_localization
