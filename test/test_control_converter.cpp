#include "akm_vehicle_interface/control_converter.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(ControlConverter, ConvertsAckermannCommandToTwist)
{
  const auto twist = akm_vehicle_interface::to_twist(1.0, 0.2, 0.32, 2.0, 0.43);

  EXPECT_DOUBLE_EQ(twist.linear.x, 1.0);
  EXPECT_NEAR(twist.angular.z, std::tan(0.2) / 0.32, 1e-9);
}

TEST(ControlConverter, ClampsSpeedAndSteering)
{
  const auto twist = akm_vehicle_interface::to_twist(3.0, 1.0, 0.32, 0.5, 0.43);

  EXPECT_DOUBLE_EQ(twist.linear.x, 0.5);
  EXPECT_NEAR(twist.angular.z, 0.5 * std::tan(0.43) / 0.32, 1e-9);
}

TEST(ControlConverter, ClampsReverseSpeed)
{
  const auto twist = akm_vehicle_interface::to_twist(-3.0, -1.0, 0.32, 0.5, 0.43);

  EXPECT_DOUBLE_EQ(twist.linear.x, -0.5);
  EXPECT_NEAR(twist.angular.z, -0.5 * std::tan(-0.43) / 0.32, 1e-9);
}
