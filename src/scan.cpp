#include "scan.hpp"

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace prior_map_localization
{
namespace
{

float read_float(const std::uint8_t * point, std::uint32_t offset)
{
  float value;
  std::memcpy(&value, point + offset, sizeof(value));
  return value;
}

std::pair<float, float> time_span(const RawScan & raw)
{
  float lo = std::numeric_limits<float>::infinity();
  float hi = -std::numeric_limits<float>::infinity();
  for (std::size_t i = 0; i < raw.count; ++i) {
    const float t = read_float(raw.data + i * raw.step, raw.time);
    if (std::isfinite(t)) {
      lo = std::min(lo, t);
      hi = std::max(hi, t);
    }
  }
  return {lo, hi};
}

// Pose after a fraction f of the motion: rotation slerped, translation linear.
Eigen::Affine3f pose_at(const ScanMotion & motion, float f)
{
  Eigen::Affine3f pose = Eigen::Affine3f::Identity();
  pose.linear() = Eigen::Quaternionf::Identity().slerp(f, motion.rotation).toRotationMatrix();
  pose.translation() = f * motion.translation;
  return pose;
}

DeskewStatus deskew_status(
  double duration_sec, const std::optional<ScanMotion> & motion, const ScanFilter & filter)
{
  if (duration_sec > filter.max_scan_duration_sec) {
    return DeskewStatus::kDurationTooLarge;
  }
  if (!motion) {
    return DeskewStatus::kNoMotion;
  }
  // The last motion stands for this sweep only when it spans about one sweep.
  const double scale = duration_sec / motion->duration_sec;
  if (!(scale > 0.0 && scale <= 1.5)) {
    return DeskewStatus::kIntervalMismatch;
  }
  return DeskewStatus::kApplied;
}

}  // namespace

std::optional<RawScan> raw_scan_fields(const FieldOffsets & fields)
{
  RawScan scan;
  int found = 0;
  for (const auto & [name, offset] : fields) {
    std::uint32_t * slot = name == "x"      ? &scan.x
                           : name == "y"    ? &scan.y
                           : name == "z"    ? &scan.z
                           : name == "time" ? &scan.time
                                            : nullptr;
    if (slot != nullptr) {
      *slot = offset;
      ++found;
    }
  }
  if (found != 4) {
    return std::nullopt;
  }
  return scan;
}

PreparedScan prepare_scan(
  const RawScan & raw, const Eigen::Affine3f & base_from_sensor,
  const std::optional<ScanMotion> & motion, const ScanFilter & filter)
{
  const auto [t_min, t_max] = time_span(raw);
  PreparedScan out;
  out.duration_sec = t_max - t_min;
  out.end_stamp_sec = raw.stamp_sec + t_max;
  out.deskew = deskew_status(out.duration_sec, motion, filter);
  const bool deskew = out.deskew == DeskewStatus::kApplied;

  // Fractions of the motion, which spans motion->duration_sec, not this sweep.
  const float per_sec = deskew ? static_cast<float>(1.0 / motion->duration_sec) : 0.0F;
  const Eigen::Affine3f end_from_motion_start =
    deskew ? pose_at(*motion, out.duration_sec * per_sec).inverse() : Eigen::Affine3f::Identity();

  Cloud::Ptr cropped(new Cloud);
  cropped->reserve(raw.count);
  for (std::size_t i = 0; i < raw.count; ++i) {
    const std::uint8_t * p = raw.data + i * raw.step;
    Eigen::Vector3f v(read_float(p, raw.x), read_float(p, raw.y), read_float(p, raw.z));
    if (!v.allFinite()) {
      continue;
    }
    v = base_from_sensor * v;
    const double range = std::hypot(v.x(), v.y());
    if (!(range > filter.min_range && range < filter.max_range)) {
      continue;
    }
    const float t = read_float(p, raw.time);
    if (deskew && std::isfinite(t)) {
      v = end_from_motion_start * (pose_at(*motion, (t - t_min) * per_sec) * v);
    }
    cropped->push_back(Point(v.x(), v.y(), v.z()));
  }

  out.cloud.reset(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(filter.voxel_leaf_size, filter.voxel_leaf_size, filter.voxel_leaf_size);
  voxel.setInputCloud(cropped);
  voxel.filter(*out.cloud);
  return out;
}

const char * to_string(DeskewStatus status)
{
  switch (status) {
    case DeskewStatus::kApplied:
      return "applied";
    case DeskewStatus::kNoMotion:
      return "no_motion";
    case DeskewStatus::kIntervalMismatch:
      return "interval_mismatch";
    case DeskewStatus::kDurationTooLarge:
      return "duration_too_large";
  }
  return "";
}

}  // namespace prior_map_localization
