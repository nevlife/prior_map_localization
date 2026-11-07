#include "registration.hpp"

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include <memory>
#include <string>

namespace prior_map_localization
{
namespace
{

// Floor and two walls, 0.2 m apart: every axis is constrained, so a true match is well posed.
Cloud::Ptr make_corner()
{
  auto cloud = std::make_shared<Cloud>();
  for (float a = 0.0f; a < 10.0f; a += 0.2f) {
    for (float b = 0.0f; b < 10.0f; b += 0.2f) {
      cloud->push_back(Point(a, b, 0.0f));
      cloud->push_back(Point(a, 0.0f, b));
      cloud->push_back(Point(0.0f, a, b));
    }
  }
  return cloud;
}

Registration make(const std::string & method, int max_iterations)
{
  Registration registration =
    method == "ndt_omp" ? make_ndt_omp({1.0, 0.1, 0.01, false}, 2, max_iterations)
                        : make_small_gicp({"GICP", 20, 2.0, 1.0, 0.0005, 0.002}, 2, max_iterations);
  registration.engine->setInputTarget(make_corner());
  return registration;
}

// The scan is the map moved by -offset, so the correct alignment is +offset in x.
Cloud::Ptr shifted_scan(float offset)
{
  auto scan = std::make_shared<Cloud>();
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  t.translation().x() = -offset;
  pcl::transformPointCloud(*make_corner(), *scan, t);
  return scan;
}

class RegistrationTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(RegistrationTest, RecoversKnownOffset)
{
  Registration r = make(GetParam(), 35);
  const Alignment a = align(r, shifted_scan(0.5f), Eigen::Matrix4f::Identity());
  ASSERT_TRUE(a.converged);
  EXPECT_NEAR(a.pose(0, 3), 0.5f, 0.05f);
  EXPECT_NEAR(a.pose(1, 3), 0.0f, 0.05f);
  EXPECT_NEAR(a.pose(2, 3), 0.0f, 0.05f);
  EXPECT_LT(a.iterations, 35);
}

// Stopping at the iteration cap is not convergence: the matcher accepts on Alignment::converged.
TEST_P(RegistrationTest, IterationCapIsNotConvergence)
{
  Registration r = make(GetParam(), 1);
  EXPECT_FALSE(align(r, shifted_scan(1.5f), Eigen::Matrix4f::Identity()).converged);
}

// An empty scan must not report the previous scan's result as a fresh convergence.
TEST_P(RegistrationTest, EmptyScanIsNotConvergence)
{
  Registration r = make(GetParam(), 35);
  ASSERT_TRUE(align(r, shifted_scan(0.5f), Eigen::Matrix4f::Identity()).converged);
  Eigen::Matrix4f seed = Eigen::Matrix4f::Identity();
  seed(1, 3) = 2.0f;
  const Alignment a = align(r, std::make_shared<Cloud>(), seed);
  EXPECT_FALSE(a.converged);
  EXPECT_TRUE(a.pose.isApprox(seed));
}

// The warm-up align must leave the engine ready to match a real scan against the same target.
TEST_P(RegistrationTest, MatchesAfterWarmup)
{
  Registration r = make(GetParam(), 35);
  set_target(r, make_corner());
  const Alignment a = align(r, shifted_scan(0.5f), Eigen::Matrix4f::Identity());
  ASSERT_TRUE(a.converged);
  EXPECT_NEAR(a.pose(0, 3), 0.5f, 0.05f);
}

INSTANTIATE_TEST_SUITE_P(Methods, RegistrationTest, ::testing::Values("ndt_omp", "small_gicp"));

TEST(WarmupProbe, TakesEveryHundredthPoint)
{
  Cloud map;
  for (int i = 0; i < 1050; ++i) {
    map.push_back(Point(static_cast<float>(i), 0.0f, 0.0f));
  }
  const Cloud::Ptr probe = warmup_probe(map);
  ASSERT_EQ(probe->size(), 105u);
  EXPECT_EQ((*probe)[1].x, 10.0f);
}

TEST(WarmupProbe, KeepsEveryPointOfASmallMap)
{
  Cloud map;
  for (int i = 0; i < 7; ++i) {
    map.push_back(Point(static_cast<float>(i), 0.0f, 0.0f));
  }
  EXPECT_EQ(warmup_probe(map)->size(), 7u);
}

}  // namespace
}  // namespace prior_map_localization
