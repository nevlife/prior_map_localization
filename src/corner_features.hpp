#ifndef PRIOR_MAP_LOCALIZATION__CORNER_FEATURES_HPP_
#define PRIOR_MAP_LOCALIZATION__CORNER_FEATURES_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace prior_map_localization
{

struct RingPoint
{
  float x;
  float y;
  float z;
  std::uint16_t ring;
};

struct CornerParams
{
  int n_scan = 0;
  int horizon_scan = 0;
  double min_range = 0.0;
  double max_range = 0.0;
  double edge_threshold = 0.0;
};

// LIO-SAM's edge points (featureExtraction, BSD-3-Clause, Tixiao Shan), the same kind the
// map bundle's corner layer holds. Sensor frame in and out; no deskew.
std::vector<Eigen::Vector3f> extract_corners(
  const std::vector<RingPoint> & points, const CornerParams & params);

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_LOCALIZATION__CORNER_FEATURES_HPP_
