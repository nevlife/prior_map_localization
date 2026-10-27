// /initialpose from one scan: a 2D branch-and-bound search of x, y and the full yaw circle
// inside the GPS circle, against the map bundle's corner layer.

#include "bbs/bbs_branch_and_bound.hpp"
#include "prior_map_localization/corner_features.hpp"

#include "sensor_msgs/flux/point_cloud2.hpp"

#include <flux/executor.hpp>
#include <flux/subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace prior_map_localization
{
namespace
{

using FluxCloud = sensor_msgs::flux_msg::PointCloud2;
constexpr auto kThrottleMs = 5000;

struct Pose2
{
  double x;
  double y;
  double yaw;
};

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

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double stamp_sec(const builtin_interfaces::msg::Time & t)
{
  return t.sec + t.nanosec * 1e-9;
}

struct RingFields
{
  std::uint32_t x;
  std::uint32_t y;
  std::uint32_t z;
  std::uint32_t ring;
};

template<typename Field>
std::optional<RingFields> ring_fields(const std::vector<Field> & fields)
{
  std::optional<std::uint32_t> x, y, z, ring;
  for (const auto & [name, offset] : fields) {
    if (name == "x") {x = offset;}
    if (name == "y") {y = offset;}
    if (name == "z") {z = offset;}
    if (name == "ring") {ring = offset;}
  }
  if (!x || !y || !z || !ring) {
    return std::nullopt;
  }
  return RingFields{*x, *y, *z, *ring};
}

std::vector<RingPoint> read_points(
  const std::uint8_t * data, std::size_t count, std::uint32_t step, const RingFields & f)
{
  std::vector<RingPoint> points(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint8_t * p = data + i * step;
    std::memcpy(&points[i].x, p + f.x, sizeof(float));
    std::memcpy(&points[i].y, p + f.y, sizeof(float));
    std::memcpy(&points[i].z, p + f.z, sizeof(float));
    std::memcpy(&points[i].ring, p + f.ring, sizeof(std::uint16_t));
  }
  return points;
}

std::vector<Eigen::Vector3f> load_xyz(const std::string & path)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(path, cloud) != 0 || cloud.empty()) {
    throw std::runtime_error("cannot load " + path);
  }
  std::vector<Eigen::Vector3f> out;
  out.reserve(cloud.size());
  for (const auto & p : cloud) {
    out.emplace_back(p.x, p.y, p.z);
  }
  return out;
}

}  // namespace

class BbsInitialPose : public rclcpp::Node
{
public:
  BbsInitialPose()
  : Node("bbs_initialpose"),
    map_frame_(declare_parameter<std::string>("map_frame")),
    base_frame_(declare_parameter<std::string>("base_frame")),
    corners_{
      static_cast<int>(declare_parameter<int>("corners.n_scan")),
      static_cast<int>(declare_parameter<int>("corners.horizon_scan")),
      declare_parameter<double>("corners.min_range"),
      declare_parameter<double>("corners.max_range"),
      declare_parameter<double>("corners.edge_threshold")},
    resolution_(declare_parameter<double>("search.resolution")),
    yaw_resolution_(declare_parameter<double>("search.yaw_resolution_deg") * M_PI / 180.0),
    pyramid_depth_(declare_parameter<int>("search.pyramid_depth")),
    min_scan_range_(declare_parameter<double>("search.min_scan_range")),
    max_scan_range_(declare_parameter<double>("search.max_scan_range")),
    max_scan_points_(declare_parameter<int>("search.max_scan_points")),
    min_score_(declare_parameter<double>("search.min_score")),
    radius_sigma_scale_(declare_parameter<double>("gps.radius_sigma_scale")),
    min_radius_(declare_parameter<double>("gps.min_radius")),
    max_radius_(declare_parameter<double>("gps.max_radius")),
    speed_margin_(declare_parameter<double>("gps.speed_margin")),
    required_votes_(declare_parameter<int>("vote.required")),
    vote_xy_tolerance_(declare_parameter<double>("vote.xy_tolerance")),
    vote_yaw_tolerance_(declare_parameter<double>("vote.yaw_tolerance_deg") * M_PI / 180.0),
    vote_timeout_sec_(declare_parameter<double>("vote.timeout_sec")),
    ground_radius_(declare_parameter<double>("ground_radius")),
    base_height_(declare_parameter<double>("base_height")),
    xy_sigma_(declare_parameter<double>("initial_xy_sigma")),
    yaw_sigma_(declare_parameter<double>("initial_yaw_sigma_deg") * M_PI / 180.0),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    load_maps(declare_parameter<std::string>("corner_pcd"), declare_parameter<std::string>("map_pcd"));
    initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/output/initialpose", 1);
    set_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/set_pose", 1);
    gps_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/gps", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry & msg) {gps_ = msg;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry & msg) {
        odom_ = Pose2{msg.pose.pose.position.x, msg.pose.pose.position.y,
          yaw_of(msg.pose.pose.orientation)};
      });
    reinit_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/input/reinitialization_requested", 10, [this](const std_msgs::msg::Bool & msg) {
        if (msg.data && !armed_) {
          RCLCPP_WARN(get_logger(), "reinitialization requested, re-arming");
          armed_ = true;
          votes_.clear();
        }
      });
    subscribe_points(declare_parameter<std::string>("transport"));
  }

  std::shared_ptr<flux::Subscription> flux_subscription() const {return flux_sub_;}

private:
  struct Vote
  {
    Pose2 map_from_odom;
    double stamp_sec;
  };

  void load_maps(const std::string & corner_pcd, const std::string & map_pcd)
  {
    const std::vector<Eigen::Vector3f> corners = load_xyz(corner_pcd);
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
    ground_ = load_xyz(map_pcd);
    RCLCPP_INFO(
      get_logger(), "corner grid %dx%d at %.2f m from %zu points", occupancy_.width,
      occupancy_.height, resolution_, corners.size());
  }

  void subscribe_points(const std::string & transport)
  {
    if (transport == "flux") {
      flux_sub_ = std::make_shared<flux::Subscription>(
        *this, "~/input/points", FluxCloud::kFingerprint, flux::QoS{},
        [this](const flux::FrameView & frame) {
          const FluxCloud::View v{frame};
          std::vector<std::pair<std::string_view, std::uint32_t>> fields;
          for (std::size_t i = 0; i < v.fields__size(); ++i) {
            fields.emplace_back(v.fields(i).name(), v.fields(i).offset());
          }
          on_points(
            fields, v.data().data(), std::size_t{v.width()} * v.height(), v.point_step(),
            v.header__sec() + v.header__nanosec() * 1e-9, std::string(v.header__frame_id()));
        });
      return;
    }
    if (transport != "dds") {
      throw std::invalid_argument("transport must be dds or flux, got " + transport);
    }
    // Depth 1: a search outlasts several scans, and only the newest is worth searching.
    dds_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input/points", rclcpp::QoS(1).reliable(),
      [this](const sensor_msgs::msg::PointCloud2 & msg) {
        std::vector<std::pair<std::string, std::uint32_t>> fields;
        for (const auto & f : msg.fields) {
          fields.emplace_back(f.name, f.offset);
        }
        on_points(
          fields, msg.data.data(), std::size_t{msg.width} * msg.height, msg.point_step,
          stamp_sec(msg.header.stamp), msg.header.frame_id);
      });
  }

  template<typename Field>
  void on_points(
    const std::vector<Field> & fields, const std::uint8_t * data, std::size_t count,
    std::uint32_t step, double stamp, const std::string & frame_id)
  {
    if (!armed_ || !gps_ || !odom_) {
      return;
    }
    const std::optional<RingFields> f = ring_fields(fields);
    if (!f) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "scan lacks x/y/z/ring");
      return;
    }
    const std::optional<Eigen::Affine3f> sensor = base_from_sensor(frame_id);
    if (!sensor) {
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    const std::vector<std::array<double, 2>> scan =
      scan_xy(extract_corners(read_points(data, count, step, *f), corners_), *sensor);

    const double gps_sigma =
      std::sqrt(std::max(gps_->pose.covariance[0], gps_->pose.covariance[7]));
    const double gps_age = std::abs(stamp - stamp_sec(gps_->header.stamp));
    const double radius = std::clamp(
      radius_sigma_scale_ * gps_sigma + speed_margin_ * gps_age, min_radius_, max_radius_);
    const double gx = gps_->pose.pose.position.x;
    const double gy = gps_->pose.pose.position.y;

    // Crop the grid to what the scan can reach from inside the circle.
    const int reach = static_cast<int>(std::ceil((radius + max_scan_range_) / resolution_));
    const int cx = static_cast<int>((gx - origin_x_) / resolution_);
    const int cy = static_cast<int>((gy - origin_y_) / resolution_);
    const int x0 = std::max(0, cx - reach);
    const int y0 = std::max(0, cy - reach);
    const int x1 = std::min(occupancy_.width - 1, cx + reach);
    const int y1 = std::min(occupancy_.height - 1, cy + reach);
    if (x1 < x0 || y1 < y0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "GPS is off the map");
      return;
    }
    bbs::Grid crop(y1 - y0 + 1, x1 - x0 + 1);
    for (int y = y0; y <= y1; ++y) {
      std::memcpy(
        &crop.data[static_cast<std::size_t>(y - y0) * crop.width],
        &occupancy_.data[static_cast<std::size_t>(y) * occupancy_.width + x0], crop.width);
    }
    const int r = static_cast<int>(std::ceil(radius / resolution_));
    const bbs::CellWindow window{cx - r - x0, cy - r - y0, cx + r - x0, cy + r - y0};
    const std::vector<bbs::BbsGridCandidate> best = bbs::branch_and_bound_candidates(
      crop, scan, resolution_, yaw_resolution_, pyramid_depth_, 2, window,
      static_cast<int>(std::ceil(2.0 / resolution_)));
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (best.empty()) {
      return;
    }
    const bbs::BbsGridCandidate & top = best[0];
    const Pose2 map_from_base{
      origin_x_ + (x0 + top.tx_cell + 0.5) * resolution_,
      origin_y_ + (y0 + top.ty_cell + 0.5) * resolution_, top.yaw_rad};
    const double runner_up = best.size() > 1 ? best[1].score : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "BBS (%.2f, %.2f) yaw %.1f deg score %.3f (next %.3f) scan %zu radius %.1f m in %.0f ms",
      map_from_base.x, map_from_base.y, map_from_base.yaw * 180.0 / M_PI, top.score, runner_up,
      scan.size(), radius, elapsed_ms);
    if (top.score < min_score_) {
      votes_.clear();
      return;
    }
    const Pose2 odom_now = *odom_;
    if (!vote(compose(map_from_base, inverse(odom_now)), stamp)) {
      return;
    }
    const Pose2 pose = compose(votes_.back().map_from_odom, odom_now);
    const std::optional<double> z = ground_z(pose.x, pose.y);
    if (!z) {
      RCLCPP_WARN(get_logger(), "no map points under (%.2f, %.2f)", pose.x, pose.y);
      votes_.clear();
      return;
    }
    publish(pose, *z, stamp);
    armed_ = false;
    votes_.clear();
  }

  // Map->odom stays put while the robot moves, so votes from different scans compare directly.
  bool vote(const Pose2 & map_from_odom, double stamp)
  {
    while (!votes_.empty() && stamp - votes_.front().stamp_sec > vote_timeout_sec_) {
      votes_.pop_front();
    }
    votes_.push_back({map_from_odom, stamp});
    int agree = 0;
    for (const Vote & v : votes_) {
      if (std::hypot(v.map_from_odom.x - map_from_odom.x, v.map_from_odom.y - map_from_odom.y) <=
        vote_xy_tolerance_ &&
        std::abs(wrap(v.map_from_odom.yaw - map_from_odom.yaw)) <= vote_yaw_tolerance_)
      {
        ++agree;
      }
    }
    return agree >= required_votes_;
  }

  std::vector<std::array<double, 2>> scan_xy(
    const std::vector<Eigen::Vector3f> & corners, const Eigen::Affine3f & base_from_sensor) const
  {
    std::set<std::pair<int, int>> cells;
    std::vector<std::array<double, 2>> out;
    for (const auto & c : corners) {
      const Eigen::Vector3f p = base_from_sensor * c;
      const double range = std::hypot(p.x(), p.y());
      if (range < min_scan_range_ || range > max_scan_range_) {
        continue;
      }
      const std::pair<int, int> cell{
        static_cast<int>(std::floor(p.x() / resolution_)),
        static_cast<int>(std::floor(p.y() / resolution_))};
      if (cells.insert(cell).second) {
        out.push_back({p.x(), p.y()});
      }
    }
    if (static_cast<int>(out.size()) > max_scan_points_) {
      std::vector<std::array<double, 2>> thinned;
      for (int i = 0; i < max_scan_points_; ++i) {
        thinned.push_back(out[i * out.size() / max_scan_points_]);
      }
      out = std::move(thinned);
    }
    return out;
  }

  std::optional<double> ground_z(double x, double y) const
  {
    std::vector<float> near;
    for (const auto & p : ground_) {
      if (std::hypot(p.x() - x, p.y() - y) < ground_radius_) {
        near.push_back(p.z());
      }
    }
    if (near.empty()) {
      return std::nullopt;
    }
    const std::size_t k = near.size() / 20;
    std::nth_element(near.begin(), near.begin() + k, near.end());
    return near[k] + base_height_;
  }

  void publish(const Pose2 & pose, double z, double stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = rclcpp::Time(static_cast<std::int64_t>(std::llround(stamp * 1e9)));
    out.header.frame_id = map_frame_;
    out.pose.pose.position.x = pose.x;
    out.pose.pose.position.y = pose.y;
    out.pose.pose.position.z = z;
    out.pose.pose.orientation.z = std::sin(pose.yaw / 2);
    out.pose.pose.orientation.w = std::cos(pose.yaw / 2);
    out.pose.covariance[0] = out.pose.covariance[7] = xy_sigma_ * xy_sigma_;
    out.pose.covariance[14] = 1.0;
    out.pose.covariance[21] = out.pose.covariance[28] = 0.01;
    out.pose.covariance[35] = yaw_sigma_ * yaw_sigma_;
    initialpose_pub_->publish(out);
    set_pose_pub_->publish(out);
    RCLCPP_INFO(
      get_logger(), "initial pose (%.2f, %.2f, z=%.2f) yaw %.1f deg", pose.x, pose.y,
      out.pose.pose.position.z, pose.yaw * 180.0 / M_PI);
  }

  std::optional<Eigen::Affine3f> base_from_sensor(const std::string & frame_id)
  {
    if (!base_from_sensor_) {
      try {
        const auto t = tf_buffer_.lookupTransform(base_frame_, frame_id, tf2::TimePointZero);
        base_from_sensor_ = tf2::transformToEigen(t).cast<float>();
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "%s", e.what());
      }
    }
    return base_from_sensor_;
  }

  std::string map_frame_;
  std::string base_frame_;
  CornerParams corners_;
  double resolution_;
  double yaw_resolution_;
  int pyramid_depth_;
  double min_scan_range_;
  double max_scan_range_;
  int max_scan_points_;
  double min_score_;
  double radius_sigma_scale_;
  double min_radius_;
  double max_radius_;
  double speed_margin_;
  int required_votes_;
  double vote_xy_tolerance_;
  double vote_yaw_tolerance_;
  double vote_timeout_sec_;
  double ground_radius_;
  double base_height_;
  double xy_sigma_;
  double yaw_sigma_;

  bbs::Grid occupancy_;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::vector<Eigen::Vector3f> ground_;

  bool armed_ = true;
  std::optional<nav_msgs::msg::Odometry> gps_;
  std::optional<Pose2> odom_;
  std::deque<Vote> votes_;
  std::optional<Eigen::Affine3f> base_from_sensor_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reinit_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr dds_sub_;
  std::shared_ptr<flux::Subscription> flux_sub_;
};

}  // namespace prior_map_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<prior_map_localization::BbsInitialPose>();
  if (const auto flux_sub = node->flux_subscription()) {
    flux::Executor executor;
    executor.add(*flux_sub);
    executor.add_node(node);
    executor.spin();
  } else {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}
