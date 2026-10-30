#include "corner_features.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace prior_map_localization
{
namespace
{

// One ring, one beam per degree.
CornerParams params() { return {1, 360, 1.0, 100.0, 1.0}; }

RingPoint at(double deg, double range)
{
  const double rad = deg * M_PI / 180.0;
  return {static_cast<float>(range * std::cos(rad)), static_cast<float>(range * std::sin(rad)),
    0.0f, 0};
}

TEST(ExtractCorners, NoneOnAConstantRange)
{
  std::vector<RingPoint> points;
  for (int deg = 0; deg < 360; ++deg) {
    points.push_back(at(deg, 10.0));
  }
  EXPECT_TRUE(extract_corners(points, params()).empty());
}

// Walls x = 5 and y = 5 seen from the origin meet at (5, 5), 45 degrees out. The vertex raises
// the curvature of the 5 beams on each side, and an earlier sector may take one of those first.
TEST(ExtractCorners, FindsTheVertexOfTwoWalls)
{
  std::vector<RingPoint> points;
  for (int deg = 0; deg <= 90; ++deg) {
    const double rad = deg * M_PI / 180.0;
    points.push_back(at(deg, deg <= 45 ? 5.0 / std::cos(rad) : 5.0 / std::sin(rad)));
  }
  const std::vector<Eigen::Vector3f> corners = extract_corners(points, params());
  ASSERT_EQ(corners.size(), 1u);
  const double deg = std::atan2(corners[0].y(), corners[0].x()) * 180.0 / M_PI;
  EXPECT_NEAR(deg, 45.0, 5.0);
}

}  // namespace
}  // namespace prior_map_localization
