#ifndef PRIOR_MAP_LOCALIZATION__SCAN_HPP_
#define PRIOR_MAP_LOCALIZATION__SCAN_HPP_

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace prior_map_localization
{

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

// Points read in place from a PointCloud2 buffer, a DDS message or a flux frame alike.
struct RawScan
{
  const std::uint8_t * data = nullptr;
  std::size_t count = 0;
  std::uint32_t step = 0;
  std::uint32_t x = 0;
  std::uint32_t y = 0;
  std::uint32_t z = 0;
  std::uint32_t time = 0;
  double stamp_sec = 0.0;
  std::string frame_id;
};

using FieldOffsets = std::vector<std::pair<std::string_view, std::uint32_t>>;

// Offsets of the float32 x, y, z and per-point time fields; nullopt when one is missing.
std::optional<RawScan> raw_scan_fields(const FieldOffsets & fields);

// Motion between the last two accepted poses, in the earlier pose's frame.
struct ScanMotion
{
  Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
  Eigen::Vector3f translation = Eigen::Vector3f::Zero();
  double duration_sec = 0.0;
};

struct ScanFilter
{
  double min_range = 0.0;
  double max_range = 0.0;
  double voxel_leaf_size = 0.0;
  // Above this the time field is not one sweep, and deskewing by it would bend the scan.
  double max_scan_duration_sec = 0.0;
};

enum class DeskewStatus { kApplied, kNoMotion, kIntervalMismatch, kDurationTooLarge };

struct PreparedScan
{
  Cloud::Ptr cloud;
  // Time of the last point: the instant every point is deskewed to and the pose is stamped at.
  double end_stamp_sec = 0.0;
  double duration_sec = 0.0;
  DeskewStatus deskew = DeskewStatus::kNoMotion;
};

// Into the base frame, cropped by horizontal range, deskewed to the scan end, then voxelized.
PreparedScan prepare_scan(
  const RawScan & raw, const Eigen::Affine3f & base_from_sensor,
  const std::optional<ScanMotion> & motion, const ScanFilter & filter);

const char * to_string(DeskewStatus status);

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_LOCALIZATION__SCAN_HPP_
