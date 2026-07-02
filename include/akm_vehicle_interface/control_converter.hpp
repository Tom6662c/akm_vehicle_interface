#ifndef AKM_VEHICLE_INTERFACE_CONTROL_CONVERTER_HPP_
#define AKM_VEHICLE_INTERFACE_CONTROL_CONVERTER_HPP_

#include "geometry_msgs/msg/twist.hpp"

#include <algorithm>
#include <cmath>

namespace akm_vehicle_interface
{

inline double clamp_symmetric(const double value, const double limit)
{
  const auto abs_limit = std::abs(limit);
  return std::clamp(value, -abs_limit, abs_limit);
}

inline geometry_msgs::msg::Twist to_twist(
  const double speed, const double steering_tire_angle, const double wheelbase,
  const double max_speed, const double max_steer_angle)
{
  geometry_msgs::msg::Twist twist;
  const auto limited_speed = clamp_symmetric(speed, max_speed);
  const auto limited_steer = clamp_symmetric(steering_tire_angle, max_steer_angle);

  twist.linear.x = limited_speed;
  twist.angular.z = limited_speed * std::tan(limited_steer) / wheelbase;
  return twist;
}

}  // namespace akm_vehicle_interface

#endif  // AKM_VEHICLE_INTERFACE_CONTROL_CONVERTER_HPP_
