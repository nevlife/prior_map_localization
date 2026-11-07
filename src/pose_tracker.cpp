#include "pose_tracker.hpp"

#include <algorithm>

namespace prior_map_localization
{
namespace
{

// About 20 s of 50 Hz odometry, longer than any prediction reaches back.
constexpr std::size_t twist_history = 1024;

}  // namespace

PoseTracker::PoseTracker(double max_prediction_dt_sec)
: max_prediction_dt_sec_(max_prediction_dt_sec)
{
}

void PoseTracker::add_twist(const TwistSample & twist)
{
  twists_.push_back(twist);
  if (twists_.size() > twist_history) {
    twists_.pop_front();
  }
}

void PoseTracker::reset(const Eigen::Matrix4f & pose, double stamp_sec)
{
  predicted_pose_ = accepted_pose_ = pose;
  predicted_stamp_sec_ = accepted_stamp_sec_ = stamp_sec;
  last_scan_accepted_ = false;
  motion_.reset();
  consecutive_rejects_ = 0;
}

// The odometry twist is in the body frame, so it moves the pose along its own axes.
Eigen::Matrix4f PoseTracker::seed(double stamp_sec) const
{
  const double dt_sec = std::clamp(stamp_sec - predicted_stamp_sec_, 0.0, max_prediction_dt_sec_);
  const auto at_or_before = std::find_if(
    twists_.rbegin(), twists_.rend(),
    [stamp_sec](const TwistSample & s) { return s.stamp_sec <= stamp_sec; });
  if (at_or_before == twists_.rend() || dt_sec <= 0.0) {
    return predicted_pose_;
  }
  const auto dt = static_cast<float>(dt_sec);
  const Eigen::Vector3f w = at_or_before->angular * dt;
  Eigen::Affine3f pose(predicted_pose_);
  pose.translation() += pose.linear() * (at_or_before->linear * dt);
  pose.linear() = pose.linear() * (Eigen::AngleAxisf(w.x(), Eigen::Vector3f::UnitX()) *
                                   Eigen::AngleAxisf(w.y(), Eigen::Vector3f::UnitY()) *
                                   Eigen::AngleAxisf(w.z(), Eigen::Vector3f::UnitZ()))
                                    .toRotationMatrix();
  return pose.matrix();
}

void PoseTracker::accept(const Eigen::Matrix4f & pose, double stamp_sec)
{
  // Deskew needs one sweep of motion, which only two accepted scans in a row measure.
  if (last_scan_accepted_) {
    const Eigen::Affine3f delta(accepted_pose_.inverse() * pose);
    motion_ = ScanMotion{
      Eigen::Quaternionf(delta.linear()).normalized(), delta.translation(),
      stamp_sec - accepted_stamp_sec_};
  }
  accepted_pose_ = predicted_pose_ = pose;
  accepted_stamp_sec_ = predicted_stamp_sec_ = stamp_sec;
  last_scan_accepted_ = true;
  consecutive_rejects_ = 0;
}

void PoseTracker::reject(const Eigen::Matrix4f & seed, double stamp_sec)
{
  predicted_pose_ = seed;
  predicted_stamp_sec_ = stamp_sec;
  last_scan_accepted_ = false;
  ++consecutive_rejects_;
}

double capped_std(double floor, double per_fitness, double max, double fitness)
{
  return std::clamp(floor + per_fitness * std::max(0.0, fitness), floor, max);
}

std::array<double, 36> pose_covariance(const PoseStd & pose_std, double fitness)
{
  const double xy =
    capped_std(pose_std.floor[0], pose_std.per_fitness[0], pose_std.max[0], fitness);
  const double z = capped_std(pose_std.floor[1], pose_std.per_fitness[1], pose_std.max[1], fitness);
  const double rp =
    capped_std(pose_std.floor[2], pose_std.per_fitness[2], pose_std.max[2], fitness);
  const double yaw =
    capped_std(pose_std.floor[3], pose_std.per_fitness[3], pose_std.max[3], fitness);
  std::array<double, 36> covariance{};
  covariance[0] = covariance[7] = xy * xy;
  covariance[14] = z * z;
  covariance[21] = covariance[28] = rp * rp;
  covariance[35] = yaw * yaw;
  return covariance;
}

}  // namespace prior_map_localization
