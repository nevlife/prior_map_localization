#include "prior_map_localization/corner_features.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace prior_map_localization
{
namespace
{

constexpr int kNeighbors = 5;
constexpr int kSectors = 6;
constexpr int kMaxEdgesPerSector = 20;
constexpr int kMaxColumnGap = 10;
constexpr float kOcclusionDepth = 0.3f;
constexpr float kParallelBeamRatio = 0.02f;

// Usable index range of one ring in the flattened image, without the curvature margins.
struct RangeImageRow
{
  int start;
  int end;
};

}  // namespace

std::vector<Eigen::Vector3f> extract_corners(
  const std::vector<RingPoint> & points, const CornerParams & params)
{
  const int rows = params.n_scan;
  const int cols = params.horizon_scan;
  std::vector<float> range_image(static_cast<std::size_t>(rows) * cols,
    std::numeric_limits<float>::max());
  std::vector<int> point_of_cell(range_image.size(), -1);
  const double column_resolution_deg = 360.0 / cols;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const RingPoint & p = points[i];
    if (p.ring >= rows) {
      continue;
    }
    const float range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (!std::isfinite(range) || range < params.min_range || range > params.max_range) {
      continue;
    }
    const double horizon_deg = std::atan2(p.x, p.y) * 180.0 / M_PI;
    int column = -static_cast<int>(std::round((horizon_deg - 90.0) / column_resolution_deg)) +
      cols / 2;
    if (column >= cols) {
      column -= cols;
    }
    if (column < 0 || column >= cols) {
      continue;
    }
    const std::size_t cell = static_cast<std::size_t>(p.ring) * cols + column;
    if (point_of_cell[cell] >= 0) {
      continue;
    }
    range_image[cell] = range;
    point_of_cell[cell] = static_cast<int>(i);
  }

  std::vector<int> index;
  std::vector<int> column_of;
  std::vector<float> range;
  std::vector<RangeImageRow> row_span(rows);
  for (int r = 0; r < rows; ++r) {
    row_span[r].start = static_cast<int>(index.size()) - 1 + kNeighbors;
    for (int c = 0; c < cols; ++c) {
      const std::size_t cell = static_cast<std::size_t>(r) * cols + c;
      if (point_of_cell[cell] >= 0) {
        index.push_back(point_of_cell[cell]);
        column_of.push_back(c);
        range.push_back(range_image[cell]);
      }
    }
    row_span[r].end = static_cast<int>(index.size()) - 1 - kNeighbors;
  }

  const int n = static_cast<int>(index.size());
  std::vector<float> curvature(n, 0.0f);
  std::vector<char> picked(n, 0);
  for (int i = kNeighbors; i < n - kNeighbors; ++i) {
    float diff = -2.0f * kNeighbors * range[i];
    for (int k = 1; k <= kNeighbors; ++k) {
      diff += range[i - k] + range[i + k];
    }
    curvature[i] = diff * diff;
  }
  for (int i = kNeighbors; i < n - kNeighbors - 1; ++i) {
    if (std::abs(column_of[i + 1] - column_of[i]) < kMaxColumnGap) {
      if (range[i] - range[i + 1] > kOcclusionDepth) {
        std::fill(picked.begin() + i - kNeighbors, picked.begin() + i + 1, 1);
      } else if (range[i + 1] - range[i] > kOcclusionDepth) {
        std::fill(picked.begin() + i + 1, picked.begin() + i + kNeighbors + 2, 1);
      }
    }
    const float d1 = std::abs(range[i - 1] - range[i]);
    const float d2 = std::abs(range[i + 1] - range[i]);
    if (d1 > kParallelBeamRatio * range[i] && d2 > kParallelBeamRatio * range[i]) {
      picked[i] = 1;
    }
  }

  std::vector<Eigen::Vector3f> corners;
  std::vector<int> order(n);
  std::iota(order.begin(), order.end(), 0);
  for (const RangeImageRow & span : row_span) {
    if (span.end <= span.start) {
      continue;
    }
    for (int s = 0; s < kSectors; ++s) {
      const int sp = (span.start * (kSectors - s) + span.end * s) / kSectors;
      const int ep = (span.start * (kSectors - 1 - s) + span.end * (s + 1)) / kSectors - 1;
      if (sp >= ep || sp < 0 || ep >= n) {
        continue;
      }
      std::sort(order.begin() + sp, order.begin() + ep,
        [&curvature](int a, int b) { return curvature[a] < curvature[b]; });
      int taken = 0;
      for (int k = ep; k >= sp; --k) {
        const int i = order[k];
        if (picked[i] || curvature[i] <= params.edge_threshold) {
          continue;
        }
        if (++taken > kMaxEdgesPerSector) {
          break;
        }
        const RingPoint & p = points[index[i]];
        corners.emplace_back(p.x, p.y, p.z);
        picked[i] = 1;
        for (int l = 1; l <= kNeighbors && i + l < n; ++l) {
          if (std::abs(column_of[i + l] - column_of[i + l - 1]) > kMaxColumnGap) {
            break;
          }
          picked[i + l] = 1;
        }
        for (int l = 1; l <= kNeighbors && i - l >= 0; ++l) {
          if (std::abs(column_of[i - l] - column_of[i - l + 1]) > kMaxColumnGap) {
            break;
          }
          picked[i - l] = 1;
        }
      }
    }
  }
  return corners;
}

}  // namespace prior_map_localization
