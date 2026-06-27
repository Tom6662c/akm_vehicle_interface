#ifndef AKM_VEHICLE_INTERFACE_HPP_
#define AKM_VEHICLE_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"

class akm_interface : public rclcpp::Node
{
public:
  akm_interface();   // 必须显式声明构造函数！

private:
  // ---------- 订阅（来自 Autoware） ----------
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_lights_cmd_sub_;

  // ---------- 发布（到 Autoware） ----------
  rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;

  // ---------- 发布到小车的 cmd_vel ----------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ---------- 订阅小车数据 ----------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // ---------- 缓存 ----------
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr latest_control_cmd_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr latest_gear_cmd_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr latest_turn_cmd_;
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr latest_hazard_cmd_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  sensor_msgs::msg::Imu::ConstSharedPtr latest_imu_;

  // ---------- 定时器 ----------
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // ---------- 车辆参数 ----------
  double wheelbase_ = 0.32;   // 轴距（米）

  // ---------- 超时标志 ----------
  bool control_timeout_ = true;

  // ---------- 回调函数 ----------
  void callback_control_cmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void callback_gear_cmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void callback_turn_indicators_cmd(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void callback_hazard_lights_cmd(const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  void callback_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  // ---------- 发布函数 ----------
  void publish_velocity_status();
  void publish_steering_status();
  void publish_gear_status();
  void publish_control_mode();
  void publish_battery_status();
  void publish_hazard_lights_status();
  void publish_turn_indicators_status();

  // ---------- 控制与超时 ----------
  void control_loop();
  void check_timeout();

  // ---------- 转换函数 ----------
  void control_command_to_vehicle(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr cmd,
    geometry_msgs::msg::Twist & twist);
};

#endif  // AKM_VEHICLE_INTERFACE_HPP_
