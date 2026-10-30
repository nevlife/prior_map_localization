#ifndef PRIOR_MAP_LOCALIZATION__GYRO_BIAS_HPP_
#define PRIOR_MAP_LOCALIZATION__GYRO_BIAS_HPP_

#include <deque>
#include <limits>
#include <optional>

namespace prior_map_localization
{

// The gyro yaw bias that LIO observes, and the check that withholds LIO when the two disagree.
// Yaw rates are about base_link z, in rad/s; stamps are seconds.
class GyroBias
{
public:
  struct Params
  {
    double time_constant_sec;
    double check_window_sec;
    double check_threshold_rad;
    double check_turn_ratio;
    double hold_sec;
  };

  explicit GyroBias(const Params & params);

  void add_imu(double stamp, double yaw_rate);
  // One LIO scan interval [t0, t1] that turned by lio_yaw_step. Returns the yaw gap when this
  // interval starts a hold, and nothing otherwise or when the IMU does not cover the interval.
  std::optional<double> add_lio(double t0, double t1, double lio_yaw_step);

  bool lio_withheld(double stamp) const { return stamp < hold_until_; }
  double bias() const { return bias_; }

private:
  struct YawSample
  {
    double stamp;
    double yaw;
    double rate;
  };
  struct YawGap
  {
    double stamp;
    double gap;
    double lio_step;
  };

  std::optional<double> imu_yaw_step(double t0, double t1) const;
  double imu_yaw_at(double stamp) const;

  Params params_;
  double bias_ = 0.0;
  double hold_until_ = -std::numeric_limits<double>::infinity();
  std::deque<YawSample> imu_yaw_;
  std::deque<YawGap> yaw_gaps_;
};

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_LOCALIZATION__GYRO_BIAS_HPP_
