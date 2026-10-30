#include "scan.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

namespace prior_map_localization
{
namespace
{

// x, y, z, time as consecutive float32 fields, the layout raw_scan_fields reads offsets for.
struct Packed
{
  std::vector<std::uint8_t> bytes;
  RawScan raw;
};

Packed pack(const std::vector<std::array<float, 4>> & points)
{
  Packed out;
  out.bytes.resize(points.size() * sizeof(points[0]));
  std::memcpy(out.bytes.data(), points.data(), out.bytes.size());
  out.raw = *raw_scan_fields({{"x", 0}, {"y", 4}, {"z", 8}, {"time", 12}});
  out.raw.data = out.bytes.data();
  out.raw.count = points.size();
  out.raw.step = sizeof(points[0]);
  return out;
}

ScanFilter filter() { return {1.0, 80.0, 0.01, 0.2}; }

std::vector<float> sorted_x(const Cloud & cloud)
{
  std::vector<float> xs;
  for (const Point & p : cloud) {
    xs.push_back(p.x);
  }
  std::sort(xs.begin(), xs.end());
  return xs;
}

TEST(RawScanFields, NeedsXYZAndTime)
{
  EXPECT_TRUE(raw_scan_fields({{"x", 0}, {"y", 4}, {"z", 8}, {"time", 12}}));
  EXPECT_FALSE(raw_scan_fields({{"x", 0}, {"y", 4}, {"z", 8}, {"intensity", 12}}));
}

TEST(PrepareScan, CropsByHorizontalRange)
{
  const Packed scan = pack({{0.5f, 0, 0, 0}, {5.0f, 0, 0, 0}, {100.0f, 0, 0, 0}});
  const PreparedScan out =
    prepare_scan(scan.raw, Eigen::Affine3f::Identity(), std::nullopt, filter());
  EXPECT_EQ(out.deskew, DeskewStatus::kNoMotion);
  EXPECT_EQ(sorted_x(*out.cloud), std::vector<float>{5.0f});
}

// Moving 1 m forward over the sweep, a wall 10 m ahead seen at the start is 9 m ahead at the end.
TEST(PrepareScan, DeskewsToTheScanEnd)
{
  const Packed scan = pack({{10.0f, 0, 0, 0.0f}, {10.0f, 0.5f, 0, 0.1f}});
  ScanMotion motion;
  motion.translation.x() = 1.0f;
  motion.duration_sec = 0.1;
  const PreparedScan out =
    prepare_scan(scan.raw, Eigen::Affine3f::Identity(), motion, filter());
  ASSERT_EQ(out.deskew, DeskewStatus::kApplied);
  const std::vector<float> xs = sorted_x(*out.cloud);
  ASSERT_EQ(xs.size(), 2u);
  EXPECT_NEAR(xs[0], 9.0f, 1e-3f);
  EXPECT_NEAR(xs[1], 10.0f, 1e-3f);
}

TEST(PrepareScan, SkipsDeskewOverALongSweep)
{
  const Packed scan = pack({{10.0f, 0, 0, 0.0f}, {10.0f, 0.5f, 0, 0.5f}});
  ScanMotion motion;
  motion.translation.x() = 1.0f;
  motion.duration_sec = 0.5;
  const PreparedScan out =
    prepare_scan(scan.raw, Eigen::Affine3f::Identity(), motion, filter());
  EXPECT_EQ(out.deskew, DeskewStatus::kDurationTooLarge);
  EXPECT_EQ(sorted_x(*out.cloud), (std::vector<float>{10.0f, 10.0f}));
}

}  // namespace
}  // namespace prior_map_localization
