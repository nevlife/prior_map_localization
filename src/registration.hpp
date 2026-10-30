#ifndef PRIOR_MAP_LOCALIZATION__REGISTRATION_HPP_
#define PRIOR_MAP_LOCALIZATION__REGISTRATION_HPP_

#include "scan.hpp"

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

struct NdtOmpParams
{
  double resolution = 0.0;
  double step_size = 0.0;
  double transformation_epsilon = 0.0;
  bool line_search = false;
};

struct SmallGicpParams
{
  std::string type;
  int num_neighbors = 0;
  double max_correspondence_distance = 0.0;
  double voxel_resolution = 0.0;
  double translation_epsilon = 0.0;
  double rotation_epsilon = 0.0;
};

Registration make_ndt_omp(const NdtOmpParams & params, int threads, int max_iterations);

Registration make_small_gicp(const SmallGicpParams & params, int threads, int max_iterations);

Alignment align(Registration & registration, const Cloud::Ptr & cloud, const Eigen::Matrix4f & seed);

}  // namespace prior_map_localization

#endif  // PRIOR_MAP_LOCALIZATION__REGISTRATION_HPP_
