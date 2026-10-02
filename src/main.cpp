#include "prior_map_localization/prior_map_matcher.hpp"

#include <cyra/spec.hpp>
#include <flux/ros/executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<prior_map_localization::PriorMapMatcher>();
  // Every scan runs on this thread, so its stage covers the whole node.
  const cyra::Report report = cyra::bind(node->get_fully_qualified_name());
  if (!report.ok()) {
    RCLCPP_WARN(node->get_logger(), "%s", report.to_string().c_str());
  }
  node->load_map();
  if (const auto flux_sub = node->flux_subscription()) {
    flux::ros::Executor executor;
    executor.add(*flux_sub);
    executor.add_ros_node(node);
    executor.spin();
  } else {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}
