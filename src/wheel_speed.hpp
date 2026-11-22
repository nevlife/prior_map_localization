#ifndef WHEEL_SPEED_HPP_
#define WHEEL_SPEED_HPP_

#include <array>

namespace prior_map_localization
{

// The [omega, v] velocity of a MINS Wheel*Cen measurement. The steering is not used, so omega is 0.
std::array<double, 2> center_wheel_velocity(double speed, double scale);

}  // namespace prior_map_localization

#endif  // WHEEL_SPEED_HPP_
