#include "gyro_bias.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <optional>

namespace prior_map_localization
{
namespace
{

constexpr double kDeg = M_PI / 180.0;

GyroBias::Params params()
{
  return {10.0, 2.0, 1.0 * kDeg, 0.03, 30.0};
}

// 100 Hz IMU and 10 Hz LIO over [t0, t1), each given its own yaw rate at time t.
// Returns the first time a hold starts, if any.
std::optional<double> drive(
  GyroBias & g, double t0, double t1, const std::function<double(double)> & imu_rate,
  const std::function<double(double)> & lio_rate)
{
  std::optional<double> first;
  for (int i = 0; t0 + i * 0.01 < t1; ++i) {
    const double t = t0 + i * 0.01;
    g.add_imu(t, imu_rate(t));
    if (i > 0 && i % 10 == 0) {
      const double started = t - 0.1;
      if (g.add_lio(started, t, lio_rate(t - 0.05) * 0.1) && !first) {
        first = t;
      }
    }
  }
  return first;
}

TEST(GyroBias, LearnsAConstantBias)
{
  GyroBias g(params());
  const auto lio = [](double t) { return 0.2 * std::sin(0.3 * t); };
  const auto imu = [&](double t) { return lio(t) + 0.001; };
  EXPECT_FALSE(drive(g, 0.0, 120.0, imu, lio));
  EXPECT_NEAR(g.bias(), 0.001, 1e-4);
}

TEST(GyroBias, WithholdsLioThatDriftsFromTheImuAndFreezesTheBias)
{
  GyroBias g(params());
  const auto straight = [](double) { return 0.0; };
  const auto imu = [](double) { return 0.001; };
  drive(g, 0.0, 60.0, imu, straight);
  const double learned = g.bias();

  const auto drifting = [](double) { return 1.0 * kDeg; };
  const std::optional<double> tripped = drive(g, 60.0, 70.0, imu, drifting);
  ASSERT_TRUE(tripped);
  EXPECT_LT(*tripped - 60.0, 2.5);
  EXPECT_TRUE(g.lio_withheld(70.0));
  // Only the scans before the trip reach the bias, and nothing after it.
  EXPECT_NEAR(g.bias(), learned, 0.003);
  const double frozen = g.bias();
  drive(g, 70.0, 80.0, imu, drifting);
  EXPECT_DOUBLE_EQ(g.bias(), frozen);
}

TEST(GyroBias, AllowsTheScaleGapOfASharpTurn)
{
  GyroBias g(params());
  const auto lio = [](double) { return 44.0 * kDeg; };
  const auto imu = [&](double t) { return 0.98 * lio(t); };
  EXPECT_FALSE(drive(g, 0.0, 4.0, imu, lio));
}

TEST(GyroBias, SkipsAnIntervalTheImuDoesNotCover)
{
  GyroBias g(params());
  g.add_imu(10.0, 0.5);
  g.add_imu(10.01, 0.5);
  EXPECT_FALSE(g.add_lio(9.9, 10.0, 0.0));
  EXPECT_EQ(g.bias(), 0.0);
  EXPECT_FALSE(g.lio_withheld(10.0));
}

}  // namespace
}  // namespace prior_map_localization
