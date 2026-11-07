#include "cov_floor.hpp"

#include <gtest/gtest.h>

#include <array>

using prior_map_localization::floor_covariance;

TEST(CovFloor, RaisesSmallVariances)
{
  const std::array<double, 9> reported{0.01, 0.5, 0, 0.5, 0.01, 0, 0, 0, 0.04};
  const auto out = floor_covariance(reported, 1.2);
  EXPECT_DOUBLE_EQ(out[0], 1.44);
  EXPECT_DOUBLE_EQ(out[4], 1.44);
  EXPECT_DOUBLE_EQ(out[8], 5.76);
}

TEST(CovFloor, KeepsLargeVariances)
{
  const std::array<double, 9> reported{9, 0, 0, 0, 16, 0, 0, 0, 25};
  const auto out = floor_covariance(reported, 1.2);
  EXPECT_DOUBLE_EQ(out[0], 9);
  EXPECT_DOUBLE_EQ(out[4], 16);
  EXPECT_DOUBLE_EQ(out[8], 25);
}

TEST(CovFloor, DropsOffDiagonal)
{
  const std::array<double, 9> reported{1, 2, 3, 4, 1, 6, 7, 8, 1};
  const auto out = floor_covariance(reported, 0.1);
  for (int i : {1, 2, 3, 5, 6, 7}) {
    EXPECT_EQ(out[i], 0.0);
  }
}
