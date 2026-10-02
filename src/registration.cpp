#include "prior_map_localization/registration.hpp"

#include <small_gicp/pcl/pcl_registration.hpp>

#include <pclomp/ndt_omp.h>

#include <stdexcept>

namespace prior_map_localization
{
namespace
{

Registration make_ndt_omp(rclcpp::Node & node, int threads, int max_iterations)
{
  auto ndt = std::make_shared<pclomp::NormalDistributionsTransform<Point, Point>>();
  ndt->setResolution(node.declare_parameter<double>("registration.ndt_omp.resolution"));
  ndt->setStepSize(node.declare_parameter<double>("registration.ndt_omp.step_size"));
  ndt->setTransformationEpsilon(
    node.declare_parameter<double>("registration.ndt_omp.transformation_epsilon"));
  ndt->setUseLineSearch(node.declare_parameter<bool>("registration.ndt_omp.line_search"));
  ndt->setNumThreads(threads);
  ndt->setMaximumIterations(max_iterations);
  return {"ndt_omp", ndt, [ndt]() { return ndt->getFinalNumIteration(); }};
}

Registration make_small_gicp(rclcpp::Node & node, int threads, int max_iterations)
{
  using SmallGicp = small_gicp::RegistrationPCL<Point, Point>;
  auto gicp = std::make_shared<SmallGicp>();
  const auto type = node.declare_parameter<std::string>("registration.small_gicp.type");
  gicp->setRegistrationType(type);
  gicp->setNumNeighborsForCovariance(
    static_cast<int>(node.declare_parameter<int>("registration.small_gicp.num_neighbors")));
  gicp->setMaxCorrespondenceDistance(
    node.declare_parameter<double>("registration.small_gicp.max_correspondence_distance"));
  gicp->setVoxelResolution(
    node.declare_parameter<double>("registration.small_gicp.voxel_resolution"));
  gicp->setTransformationEpsilon(
    node.declare_parameter<double>("registration.small_gicp.translation_epsilon"));
  gicp->setRotationEpsilon(
    node.declare_parameter<double>("registration.small_gicp.rotation_epsilon"));
  gicp->setNumThreads(threads);
  gicp->setMaximumIterations(max_iterations);
  return {"small_gicp_" + type, gicp, [gicp]() {
            return static_cast<int>(gicp->getRegistrationResult().iterations);
          }};
}

}  // namespace

Registration make_registration(rclcpp::Node & node)
{
  const auto method = node.declare_parameter<std::string>("registration.method");
  const auto threads = static_cast<int>(node.declare_parameter<int>("registration.num_threads"));
  const auto max_iterations =
    static_cast<int>(node.declare_parameter<int>("registration.max_iterations"));
  if (method == "ndt_omp") {
    return make_ndt_omp(node, threads, max_iterations);
  }
  if (method == "small_gicp") {
    return make_small_gicp(node, threads, max_iterations);
  }
  throw std::invalid_argument("registration.method must be ndt_omp or small_gicp, got " + method);
}

}  // namespace prior_map_localization
