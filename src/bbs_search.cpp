#include "bbs_search.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <utility>
#include <vector>

namespace prior_map_localization
{
namespace
{

// The best match and the runner-up, which the log reports to show how distinct the best is.
constexpr int bbs_candidates = 2;
constexpr double bbs_nms_radius_m = 2.0;
// The 5th percentile of the map z near a point.
constexpr std::size_t ground_percentile_divisor = 20;

}  // namespace

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

Pose2 compose(const Pose2 & a, const Pose2 & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap(a.yaw + b.yaw)};
}

Pose2 inverse(const Pose2 & a)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {-c * a.x - s * a.y, s * a.x - c * a.y, -a.yaw};
}

double search_radius(
  const GpsRadiusParams & params, double cov_xx, double cov_yy, double scan_stamp, double gps_stamp)
{
  const double gps_sigma = std::sqrt(std::max(cov_xx, cov_yy));
  const double gps_age = std::abs(scan_stamp - gps_stamp);
  return std::clamp(
    params.sigma_scale * gps_sigma + params.speed_margin * gps_age, params.min_radius,
    params.max_radius);
}

std::vector<std::array<double, 2>> scan_xy(
  const std::vector<Eigen::Vector3f> & corners, const Eigen::Affine3f & base_from_sensor,
  const BbsParams & params)
{
  std::set<std::pair<int, int>> cells;
  std::vector<std::array<double, 2>> out;
  for (const auto & c : corners) {
    const Eigen::Vector3f p = base_from_sensor * c;
    const double range = std::hypot(p.x(), p.y());
    if (range < params.min_scan_range || range > params.max_scan_range) {
      continue;
    }
    const std::pair<int, int> cell{
      static_cast<int>(std::floor(p.x() / params.resolution)),
      static_cast<int>(std::floor(p.y() / params.resolution))};
    if (cells.insert(cell).second) {
      out.push_back({p.x(), p.y()});
    }
  }
  if (static_cast<int>(out.size()) > params.max_scan_points) {
    std::vector<std::array<double, 2>> thinned;
    for (int i = 0; i < params.max_scan_points; ++i) {
      thinned.push_back(out[i * out.size() / params.max_scan_points]);
    }
    out = std::move(thinned);
  }
  return out;
}

CornerGrid::CornerGrid(const std::vector<Eigen::Vector3f> & corners, double resolution)
: resolution_(resolution)
{
  float min_x = corners[0].x(), min_y = corners[0].y();
  float max_x = min_x, max_y = min_y;
  for (const auto & p : corners) {
    min_x = std::min(min_x, p.x());
    min_y = std::min(min_y, p.y());
    max_x = std::max(max_x, p.x());
    max_y = std::max(max_y, p.y());
  }
  origin_x_ = min_x;
  origin_y_ = min_y;
  occupancy_ = bbs::Grid(
    static_cast<int>((max_y - min_y) / resolution_) + 1,
    static_cast<int>((max_x - min_x) / resolution_) + 1);
  for (const auto & p : corners) {
    occupancy_.set(
      static_cast<int>((p.y() - min_y) / resolution_),
      static_cast<int>((p.x() - min_x) / resolution_), 1);
  }
}

std::optional<GridCrop> CornerGrid::crop(
  double gx, double gy, double radius, double max_scan_range) const
{
  const int reach = static_cast<int>(std::ceil((radius + max_scan_range) / resolution_));
  const int cx = static_cast<int>((gx - origin_x_) / resolution_);
  const int cy = static_cast<int>((gy - origin_y_) / resolution_);
  const int x0 = std::max(0, cx - reach);
  const int y0 = std::max(0, cy - reach);
  const int x1 = std::min(occupancy_.width - 1, cx + reach);
  const int y1 = std::min(occupancy_.height - 1, cy + reach);
  if (x1 < x0 || y1 < y0) {
    return std::nullopt;
  }
  bbs::Grid grid(y1 - y0 + 1, x1 - x0 + 1);
  for (int y = y0; y <= y1; ++y) {
    std::memcpy(
      &grid.data[static_cast<std::size_t>(y - y0) * grid.width],
      &occupancy_.data[static_cast<std::size_t>(y) * occupancy_.width + x0], grid.width);
  }
  const int r = static_cast<int>(std::ceil(radius / resolution_));
  const bbs::CellWindow window{cx - r - x0, cy - r - y0, cx + r - x0, cy + r - y0};
  return GridCrop{std::move(grid), x0, y0, window};
}

std::optional<BbsMatch> CornerGrid::search(
  const GridCrop & crop, const std::vector<std::array<double, 2>> & scan,
  const BbsParams & params) const
{
  const std::vector<bbs::BbsGridCandidate> best = bbs::branch_and_bound_candidates(
    crop.grid, scan, resolution_, params.yaw_resolution, params.pyramid_depth, bbs_candidates,
    crop.window, static_cast<int>(std::ceil(bbs_nms_radius_m / resolution_)));
  if (best.empty()) {
    return std::nullopt;
  }
  const bbs::BbsGridCandidate & top = best[0];
  const Pose2 map_from_base{
    origin_x_ + (crop.x0 + top.tx_cell + 0.5) * resolution_,
    origin_y_ + (crop.y0 + top.ty_cell + 0.5) * resolution_, top.yaw_rad};
  return BbsMatch{map_from_base, top.score, best.size() > 1 ? best[1].score : 0.0};
}

PoseVote::PoseVote(const Params & params) : params_(params)
{
}

bool PoseVote::add(const Pose2 & map_from_odom, double stamp)
{
  while (!votes_.empty() && stamp - votes_.front().stamp_sec > params_.timeout_sec) {
    votes_.pop_front();
  }
  votes_.push_back({map_from_odom, stamp});
  int agree = 0;
  for (const Vote & v : votes_) {
    if (
      std::hypot(v.map_from_odom.x - map_from_odom.x, v.map_from_odom.y - map_from_odom.y) <=
        params_.xy_tolerance &&
      std::abs(wrap(v.map_from_odom.yaw - map_from_odom.yaw)) <= params_.yaw_tolerance) {
      ++agree;
    }
  }
  return agree >= params_.required;
}

GroundHeight::GroundHeight(std::vector<Eigen::Vector3f> points, double radius)
: points_(std::move(points)), radius_(radius)
{
}

std::optional<double> GroundHeight::at(double x, double y) const
{
  std::vector<float> near;
  for (const auto & p : points_) {
    if (std::hypot(p.x() - x, p.y() - y) < radius_) {
      near.push_back(p.z());
    }
  }
  if (near.empty()) {
    return std::nullopt;
  }
  const std::size_t k = near.size() / ground_percentile_divisor;
  std::nth_element(near.begin(), near.begin() + k, near.end());
  return near[k];
}

}  // namespace prior_map_localization
