#ifndef POSE_TRACKER_HPP_
#define POSE_TRACKER_HPP_

#include "scan.hpp"

#include <Eigen/Geometry>

#include <array>
#include <deque>
#include <optional>
#include <vector>

namespace prior_map_localization
{

struct TwistSample
{
  double stamp_sec;
  Eigen::Vector3f linear;
  Eigen::Vector3f angular;
};

// The matcher's pose between scans: predicted from the odometry twist, reset by an accepted scan.
class PoseTracker
{
public:
  explicit PoseTracker(double max_prediction_dt_sec);

  void add_twist(const TwistSample & twist);
  void reset(const Eigen::Matrix4f & pose, double stamp_sec);
  // The registration seed at the scan end.
  Eigen::Matrix4f seed(double stamp_sec) const;
  void accept(const Eigen::Matrix4f & pose, double stamp_sec);
  // A rejected scan still moves the prediction on, so the next seed starts from here.
  void reject(const Eigen::Matrix4f & seed, double stamp_sec);

  const std::optional<ScanMotion> & motion() const { return motion_; }
  double accepted_stamp_sec() const { return accepted_stamp_sec_; }
  int consecutive_rejects() const { return consecutive_rejects_; }

private:
  double max_prediction_dt_sec_;
  std::deque<TwistSample> twists_;
  Eigen::Matrix4f predicted_pose_ = Eigen::Matrix4f::Identity();
  double predicted_stamp_sec_ = 0.0;
  Eigen::Matrix4f accepted_pose_ = Eigen::Matrix4f::Identity();
  double accepted_stamp_sec_ = 0.0;
  bool last_scan_accepted_ = false;
  std::optional<ScanMotion> motion_;
  int consecutive_rejects_ = 0;
};

// [xy, z, roll and pitch, yaw] standard deviations: floor + per_fitness * fitness, capped.
struct PoseStd
{
  std::vector<double> floor;
  std::vector<double> per_fitness;
  std::vector<double> max;
};

double capped_std(double floor, double per_fitness, double max, double fitness);
std::array<double, 36> pose_covariance(const PoseStd & pose_std, double fitness);

}  // namespace prior_map_localization

#endif  // POSE_TRACKER_HPP_
