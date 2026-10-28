#ifndef PRIOR_MAP_LOCALIZATION__REGISTRATION_HPP_
#define PRIOR_MAP_LOCALIZATION__REGISTRATION_HPP_

#include "prior_map_localization/scan.hpp"

#include <rclcpp/node.hpp>

#include <pcl/registration/registration.h>

#include <Eigen/Core>

#include <functional>
#include <memory>
#include <string>

namespace prior_map_localization
{

// One scan-to-map engine behind pcl::Registration, so ndt_omp and small_gicp swap on one bag.
struct Registration
{
  std::string method;
  std::shared_ptr<pcl::Registration<Point, Point, float>> engine;
  // pcl::Registration keeps the count protected; each engine exposes it its own way.
  std::function<int()> iterations;
};

struct Alignment
{
  Eigen::Matrix4f pose;
  bool converged;
  double fitness;
  double time_sec;
  int iterations;
};

// Declares the registration.* parameters of the selected method only.
Registration make_registration(rclcpp::Node & node);

Alignment align(Registration & registration, const Cloud::Ptr & cloud, const Eigen::Matrix4f & seed);

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_LOCALIZATION__REGISTRATION_HPP_
