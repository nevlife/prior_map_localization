#include "wheel_speed.hpp"

#include <gtest/gtest.h>

using prior_map_localization::center_wheel_velocity;

TEST(CenterWheelVelocity, ScalesTheSpeed)
{
  const auto v = center_wheel_velocity(2.0, 0.935);
  EXPECT_DOUBLE_EQ(v[1], 1.87);
}

TEST(CenterWheelVelocity, HasNoYawRate)
{
  EXPECT_EQ(center_wheel_velocity(1.5, 0.935)[0], 0.0);
  EXPECT_EQ(center_wheel_velocity(-1.5, 0.935)[0], 0.0);
}

TEST(CenterWheelVelocity, KeepsReverseNegative)
{
  EXPECT_DOUBLE_EQ(center_wheel_velocity(-1.0, 0.5)[1], -0.5);
}
