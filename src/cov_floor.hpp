#ifndef COV_FLOOR_HPP_
#define COV_FLOOR_HPP_

#include <array>

namespace prior_map_localization
{

// A diagonal position covariance whose sigma is at least floor horizontally and 2 * floor
// vertically. Off-diagonal terms are dropped.
std::array<double, 9> floor_covariance(const std::array<double, 9> & reported, double floor);

}  // namespace prior_map_localization

#endif  // COV_FLOOR_HPP_
