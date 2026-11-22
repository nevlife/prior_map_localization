#include "wheel_speed.hpp"

namespace prior_map_localization
{

std::array<double, 2> center_wheel_velocity(double speed, double scale)
{
  return {0.0, speed * scale};
}

}  // namespace prior_map_localization
