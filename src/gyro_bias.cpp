#include "gyro_bias.hpp"

#include <cmath>

namespace prior_map_localization
{
namespace
{

// The IMU yaw is kept this long, longer than any LIO interval or check window.
constexpr double kImuHistorySec = 10.0;

}  // namespace

GyroBias::GyroBias(const Params & params) : params_(params) {}

void GyroBias::add_imu(double stamp, double yaw_rate)
{
  double yaw = 0.0;
  if (!imu_yaw_.empty()) {
    const YawSample & last = imu_yaw_.back();
    yaw = last.yaw + 0.5 * (yaw_rate + last.rate) * (stamp - last.stamp);
  }
  imu_yaw_.push_back({stamp, yaw, yaw_rate});
  while (imu_yaw_.front().stamp < stamp - kImuHistorySec) {
    imu_yaw_.pop_front();
  }
}

std::optional<double> GyroBias::add_lio(double t0, double t1, double lio_yaw_step)
{
  const std::optional<double> imu_step = imu_yaw_step(t0, t1);
  if (!imu_step) {
    return std::nullopt;
  }
  const double dt = t1 - t0;
  yaw_gaps_.push_back({t1, lio_yaw_step - (*imu_step - bias_ * dt), lio_yaw_step});
  while (yaw_gaps_.front().stamp < t1 - params_.check_window_sec) {
    yaw_gaps_.pop_front();
  }
  double gap = 0.0;
  double turn = 0.0;
  for (const YawGap & g : yaw_gaps_) {
    gap += g.gap;
    turn += g.lio_step;
  }
  std::optional<double> started;
  // The IMU and LIO yaw scales differ by about 1 %, so a sharp turn opens a gap by itself.
  if (std::abs(gap) > params_.check_threshold_rad + params_.check_turn_ratio * std::abs(turn)) {
    if (!lio_withheld(t1)) {
      started = gap;
    }
    hold_until_ = t1 + params_.hold_sec;
  }
  if (!lio_withheld(t1)) {
    bias_ += dt / params_.time_constant_sec * ((*imu_step - lio_yaw_step) / dt - bias_);
  }
  return started;
}

std::optional<double> GyroBias::imu_yaw_step(double t0, double t1) const
{
  if (imu_yaw_.empty() || imu_yaw_.front().stamp > t0 || imu_yaw_.back().stamp < t1) {
    return std::nullopt;
  }
  return imu_yaw_at(t1) - imu_yaw_at(t0);
}

double GyroBias::imu_yaw_at(double stamp) const
{
  for (std::size_t i = 1; i < imu_yaw_.size(); ++i) {
    const YawSample & a = imu_yaw_[i - 1];
    const YawSample & b = imu_yaw_[i];
    if (b.stamp >= stamp) {
      const double span = b.stamp - a.stamp;
      return span > 0.0 ? a.yaw + (b.yaw - a.yaw) * (stamp - a.stamp) / span : b.yaw;
    }
  }
  return imu_yaw_.back().yaw;
}

}  // namespace prior_map_localization
