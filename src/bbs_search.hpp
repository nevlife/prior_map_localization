#ifndef BBS_SEARCH_HPP_
#define BBS_SEARCH_HPP_

#include "bbs/bbs_branch_and_bound.hpp"

#include <Eigen/Geometry>

#include <array>
#include <deque>
#include <optional>
#include <vector>

namespace prior_map_localization
{

struct Pose2
{
  double x;
  double y;
  double yaw;
};

double wrap(double a);
Pose2 compose(const Pose2 & a, const Pose2 & b);
Pose2 inverse(const Pose2 & a);

struct BbsParams
{
  double resolution;
  double yaw_resolution;
  int pyramid_depth;
  double min_scan_range;
  double max_scan_range;
  int max_scan_points;
};

struct GpsRadiusParams
{
  double sigma_scale;
  double min_radius;
  double max_radius;
  double speed_margin;
};

// The search circle around the GPS fix: its sigma, widened by how far the robot can move
// between the fix and the scan.
double search_radius(
  const GpsRadiusParams & params, double cov_xx, double cov_yy, double scan_stamp,
  double gps_stamp);

// Base-frame corner xy within the scan range, one per grid cell, thinned evenly to the cap.
std::vector<std::array<double, 2>> scan_xy(
  const std::vector<Eigen::Vector3f> & corners, const Eigen::Affine3f & base_from_sensor,
  const BbsParams & params);

struct GridCrop
{
  bbs::Grid grid;
  int x0;
  int y0;
  bbs::CellWindow window;
};

struct BbsMatch
{
  Pose2 map_from_base;
  double score;
  double runner_up;
};

// The map's corner layer as an occupancy grid whose origin is the corners' xy minimum.
class CornerGrid
{
public:
  CornerGrid(const std::vector<Eigen::Vector3f> & corners, double resolution);

  // The grid cropped to what a scan reaches from inside the circle; nullopt when that is off the
  // map.
  std::optional<GridCrop> crop(double gx, double gy, double radius, double max_scan_range) const;
  std::optional<BbsMatch> search(
    const GridCrop & crop, const std::vector<std::array<double, 2>> & scan,
    const BbsParams & params) const;

  int width() const { return occupancy_.width; }
  int height() const { return occupancy_.height; }

private:
  double resolution_;
  double origin_x_;
  double origin_y_;
  bbs::Grid occupancy_;
};

// Agreeing map->odom estimates within the timeout. Map->odom stays put while the robot moves,
// so votes from different scans compare directly.
class PoseVote
{
public:
  struct Params
  {
    int required;
    double xy_tolerance;
    double yaw_tolerance;
    double timeout_sec;
  };

  explicit PoseVote(const Params & params);

  bool add(const Pose2 & map_from_odom, double stamp);
  void clear() { votes_.clear(); }

private:
  struct Vote
  {
    Pose2 map_from_odom;
    double stamp_sec;
  };

  Params params_;
  std::deque<Vote> votes_;
};

// The ground height under a point: a low percentile of the map z within the radius.
class GroundHeight
{
public:
  GroundHeight(std::vector<Eigen::Vector3f> points, double radius);

  std::optional<double> at(double x, double y) const;

private:
  std::vector<Eigen::Vector3f> points_;
  double radius_;
};

}  // namespace prior_map_localization

#endif  // BBS_SEARCH_HPP_
