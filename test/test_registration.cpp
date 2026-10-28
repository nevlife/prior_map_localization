#include "prior_map_localization/registration.hpp"

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

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
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"registration.method", method},
    {"registration.num_threads", 2},
    {"registration.max_iterations", max_iterations},
    {"registration.ndt_omp.resolution", 1.0},
    {"registration.ndt_omp.step_size", 0.1},
    {"registration.ndt_omp.transformation_epsilon", 0.01},
    {"registration.ndt_omp.line_search", false},
    {"registration.small_gicp.type", std::string("GICP")},
    {"registration.small_gicp.num_neighbors", 20},
    {"registration.small_gicp.max_correspondence_distance", 2.0},
    {"registration.small_gicp.voxel_resolution", 1.0},
    {"registration.small_gicp.translation_epsilon", 0.0005},
    {"registration.small_gicp.rotation_epsilon", 0.002},
  });
  rclcpp::Node node("test_registration", options);
  Registration registration = make_registration(node);
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
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
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

INSTANTIATE_TEST_SUITE_P(Methods, RegistrationTest, ::testing::Values("ndt_omp", "small_gicp"));

}  // namespace
}  // namespace prior_map_localization
