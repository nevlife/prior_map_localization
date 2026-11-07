#include "cov_floor.hpp"

#include <algorithm>

namespace prior_map_localization
{

std::array<double, 9> floor_covariance(const std::array<double, 9> & reported, double floor)
{
  const double variance = floor * floor;
  std::array<double, 9> out{};
  out[0] = std::max(reported[0], variance);
  out[4] = std::max(reported[4], variance);
  out[8] = std::max(reported[8], 4.0 * variance);
  return out;
}

}  // namespace prior_map_localization
