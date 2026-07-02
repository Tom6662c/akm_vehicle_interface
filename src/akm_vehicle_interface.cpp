#include "akm_vehicle_interface/akm_vehicle_interface.hpp"

#include "akm_vehicle_interface/control_converter.hpp"

#include <chrono>

akm_interface::akm_interface()
: Node("akm_interface")
{
  using std::placeholders::_1;

  wheelbase_ = declare_parameter<double>("wheelbase", wheelbase_);
  control_timeout_sec_ = declare_parameter<double>("control_timeout_sec", control_timeout_sec_);
  max_speed_ = declare_parameter<double>("max_speed", max_speed_);
  max_steer_angle_ = declare_parameter<double>("max_steer_angle", max_steer_angle_);
  input_control_cmd_topic_ =
    declare_parameter<std::string>("input_control_cmd_topic", input_control_cmd_topic_);
  output_cmd_vel_topic_ =
    declare_parameter<std::string>("output_cmd_vel_topic", output_cmd_vel_topic_);
  input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", input_odom_topic_);
  input_imu_topic_ = declare_parameter<std::string>("input_imu_topic", input_imu_topic_);

  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    input_control_cmd_topic_, 1, std::bind(&akm_interface::callback_control_cmd, this, _1));
  gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&akm_interface::callback_gear_cmd, this, _1));
  turn_indicators_cmd_sub_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "/control/command/turn_indicators_cmd", 1,
    std::bind(&akm_interface::callback_turn_indicators_cmd, this, _1));
  hazard_lights_cmd_sub_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
    "/control/command/hazard_lights_cmd", 1,
    std::bind(&akm_interface::callback_hazard_lights_cmd, this, _1));

  velocity_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  steering_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  control_mode_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  battery_pub_ = create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>(
    "/vehicle/status/battery_charge", rclcpp::QoS{1});
  hazard_lights_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
  turn_indicators_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
    "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    output_cmd_vel_topic_, rclcpp::QoS{1});

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_, 1, std::bind(&akm_interface::callback_odom, this, _1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    input_imu_topic_, 1, std::bind(&akm_interface::callback_imu, this, _1));

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&akm_interface::control_loop, this));
  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&akm_interface::check_timeout, this));

  RCLCPP_INFO(
    get_logger(),
    "akm_vehicle_interface started: wheelbase=%.3f max_speed=%.3f max_steer=%.3f "
    "timeout=%.3f control_cmd=%s cmd_vel=%s odom=%s imu=%s",
    wheelbase_, max_speed_, max_steer_angle_, control_timeout_sec_,
    input_control_cmd_topic_.c_str(), output_cmd_vel_topic_.c_str(), input_odom_topic_.c_str(),
    input_imu_topic_.c_str());
}

void akm_interface::callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  latest_imu_ = msg;
  last_imu_received_time_ = now();
}

void akm_interface::callback_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  latest_odom_ = msg;
  last_odom_received_time_ = now();
  publish_velocity_status();
  publish_steering_status();
  publish_battery_status();
  publish_hazard_lights_status();
  publish_turn_indicators_status();
  publish_gear_status();
  publish_control_mode();
}

void akm_interface::publish_steering_status()
{
  autoware_auto_vehicle_msgs::msg::SteeringReport msg;
  msg.stamp = now();
  msg.steering_tire_angle = latest_control_cmd_ ?
    akm_vehicle_interface::clamp_symmetric(
    latest_control_cmd_->lateral.steering_tire_angle, max_steer_angle_) :
    0.0;
  steering_status_pub_->publish(msg);
}

void akm_interface::publish_gear_status()
{
  autoware_auto_vehicle_msgs::msg::GearReport msg;
  msg.stamp = now();
  msg.report = autoware_auto_vehicle_msgs::msg::GearReport::DRIVE;
  gear_status_pub_->publish(msg);
}

void akm_interface::publish_control_mode()
{
  autoware_auto_vehicle_msgs::msg::ControlModeReport msg;
  msg.stamp = now();
  msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  control_mode_pub_->publish(msg);
}

void akm_interface::publish_velocity_status()
{
  if (!latest_odom_) {
    return;
  }

  autoware_auto_vehicle_msgs::msg::VelocityReport msg;
  msg.header.stamp = latest_odom_->header.stamp;
  msg.header.frame_id = "base_link";
  msg.longitudinal_velocity = latest_odom_->twist.twist.linear.x;
  msg.lateral_velocity = latest_odom_->twist.twist.linear.y;
  msg.heading_rate = latest_odom_->twist.twist.angular.z;
  velocity_status_pub_->publish(msg);
}

void akm_interface::publish_battery_status()
{
  tier4_vehicle_msgs::msg::BatteryStatus msg;
  msg.stamp = now();
  msg.energy_level = 100.0;
  battery_pub_->publish(msg);
}

void akm_interface::publish_hazard_lights_status()
{
  autoware_auto_vehicle_msgs::msg::HazardLightsReport msg;
  msg.stamp = now();
  msg.report = latest_hazard_cmd_ ?
    latest_hazard_cmd_->command :
    autoware_auto_vehicle_msgs::msg::HazardLightsReport::DISABLE;
  hazard_lights_pub_->publish(msg);
}

void akm_interface::publish_turn_indicators_status()
{
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport msg;
  msg.stamp = now();
  msg.report = latest_turn_cmd_ ?
    latest_turn_cmd_->command :
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
  turn_indicators_pub_->publish(msg);
}

void akm_interface::callback_control_cmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  latest_control_cmd_ = msg;
  last_control_cmd_received_time_ = now();
  control_timeout_ = false;

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "received control_cmd speed=%.3f steer=%.3f",
    msg->longitudinal.speed, msg->lateral.steering_tire_angle);
}

void akm_interface::callback_gear_cmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  latest_gear_cmd_ = msg;
}

void akm_interface::callback_turn_indicators_cmd(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  latest_turn_cmd_ = msg;
}

void akm_interface::callback_hazard_lights_cmd(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  latest_hazard_cmd_ = msg;
}

void akm_interface::control_loop()
{
  if (!latest_control_cmd_ || control_timeout_) {
    geometry_msgs::msg::Twist zero_cmd;
    cmd_vel_pub_->publish(zero_cmd);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "control command timeout or missing; publishing zero cmd_vel");
    return;
  }

  geometry_msgs::msg::Twist cmd_vel;
  control_command_to_vehicle(latest_control_cmd_, cmd_vel);
  cmd_vel_pub_->publish(cmd_vel);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "published cmd_vel linear_x=%.3f angular_z=%.3f", cmd_vel.linear.x, cmd_vel.angular.z);
}

void akm_interface::control_command_to_vehicle(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr cmd,
  geometry_msgs::msg::Twist & twist)
{
  twist = akm_vehicle_interface::to_twist(
    cmd->longitudinal.speed, cmd->lateral.steering_tire_angle, wheelbase_, max_speed_,
    max_steer_angle_);
}

void akm_interface::check_timeout()
{
  if (!latest_control_cmd_) {
    control_timeout_ = true;
  } else {
    const auto elapsed = (now() - last_control_cmd_received_time_).seconds();
    control_timeout_ = elapsed > control_timeout_sec_;
  }

  if (!latest_odom_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "no odom received from %s", input_odom_topic_.c_str());
  } else if ((now() - last_odom_received_time_).seconds() > control_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "odom data timeout from %s", input_odom_topic_.c_str());
  }

  if (!latest_imu_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "no imu received from %s", input_imu_topic_.c_str());
  } else if ((now() - last_imu_received_time_).seconds() > control_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "imu data timeout from %s", input_imu_topic_.c_str());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<akm_interface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
