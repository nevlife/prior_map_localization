#include "registration.hpp"

#include <small_gicp/pcl/pcl_registration.hpp>

#include <pclomp/ndt_omp.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>

namespace prior_map_localization
{
namespace
{

constexpr std::size_t warmup_probe_points = 100;

}  // namespace

Registration make_ndt_omp(const NdtOmpParams & params, int threads, int max_iterations)
{
  auto ndt = std::make_shared<pclomp::NormalDistributionsTransform<Point, Point>>();
  ndt->setResolution(params.resolution);
  ndt->setStepSize(params.step_size);
  ndt->setTransformationEpsilon(params.transformation_epsilon);
  ndt->setUseLineSearch(params.line_search);
  ndt->setNumThreads(threads);
  ndt->setMaximumIterations(max_iterations);
  return {"ndt_omp", ndt, [ndt]() { return ndt->getFinalNumIteration(); }};
}

Registration make_small_gicp(const SmallGicpParams & params, int threads, int max_iterations)
{
  using SmallGicp = small_gicp::RegistrationPCL<Point, Point>;
  auto gicp = std::make_shared<SmallGicp>();
  gicp->setRegistrationType(params.type);
  gicp->setNumNeighborsForCovariance(params.num_neighbors);
  gicp->setMaxCorrespondenceDistance(params.max_correspondence_distance);
  gicp->setVoxelResolution(params.voxel_resolution);
  gicp->setTransformationEpsilon(params.translation_epsilon);
  gicp->setRotationEpsilon(params.rotation_epsilon);
  gicp->setNumThreads(threads);
  gicp->setMaximumIterations(max_iterations);
  return {"small_gicp_" + params.type, gicp, [gicp]() {
            return static_cast<int>(gicp->getRegistrationResult().iterations);
          }};
}

Alignment align(Registration & registration, const Cloud::Ptr & cloud, const Eigen::Matrix4f & seed)
{
  // pcl::Registration::align returns early on an empty source and keeps the previous result.
  if (cloud->empty()) {
    return {seed, false, std::numeric_limits<double>::quiet_NaN(), 0.0, 0};
  }
  auto & engine = *registration.engine;
  engine.setInputSource(cloud);
  Cloud aligned;
  const auto start = std::chrono::steady_clock::now();
  engine.align(aligned, seed);
  const double time_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  return {
    engine.getFinalTransformation(), engine.hasConverged(), engine.getFitnessScore(), time_sec,
    registration.iterations()};
}

Cloud::Ptr warmup_probe(const Cloud & map)
{
  Cloud::Ptr probe(new Cloud);
  const std::size_t step = std::max<std::size_t>(1, map.size() / warmup_probe_points);
  for (std::size_t i = 0; i < map.size(); i += step) {
    probe->push_back(map[i]);
  }
  return probe;
}

void set_target(Registration & registration, const Cloud::Ptr & map)
{
  registration.engine->setInputTarget(map);
  Cloud aligned;
  registration.engine->setInputSource(warmup_probe(*map));
  registration.engine->align(aligned, Eigen::Matrix4f::Identity());
}

}  // namespace prior_map_localization
