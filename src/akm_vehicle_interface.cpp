// akm_vehicle_interface.cpp
#include "akm_vehicle_interface/akm_vehicle_interface.hpp"
#include <cmath>
akm_interface::akm_interface() : Node("akm_interface")
{
    using std::placeholders::_1;

    // ===== 订阅 Autoware 控制指令 =====
    control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", 1,
        std::bind(&akm_interface::callback_control_cmd, this, _1));
    
    gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
        "/control/command/gear_cmd", 1,
        std::bind(&akm_interface::callback_gear_cmd, this, _1));
    turn_indicators_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
        "/control/command/turn_indicators_cmd", 1,
        std::bind(&akm_interface::callback_turn_indicators_cmd, this, std::placeholders::_1));
    hazard_lights_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
        "/control/command/hazard_lights_cmd", 1,
        std::bind(&akm_interface::callback_hazard_lights_cmd, this, std::placeholders::_1));
    // ... 其他订阅

    // ===== 发布到 Autoware 的状态 =====
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
    // ... 其他发布

    // ===== 发布到小车的 cmd_vel =====
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS{1});

    // ===== 订阅小车数据 =====
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&akm_interface::callback_odom, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_raw", 1,
        std::bind(&akm_interface::callback_imu, this, _1));
        
    // === 定时器：定期发布控制模式、检查超时、执行控制指令 ===
    // 控制执行定时器（50Hz，即20ms周期）
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&akm_interface::control_loop, this));

    // 超时检查定时器（10Hz）
    timeout_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&akm_interface::check_timeout, this));
}


void akm_interface::callback_imu(
    const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    latest_imu_ = msg;
    // 可在此提取角速度、加速度等，用于补充转向或姿态
}

void akm_interface::callback_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    latest_odom_ = msg;
    // 每次收到里程计，更新所有状态话题
    publish_velocity_status();
    publish_steering_status();
    publish_battery_status();      // 可定期发布，不一定每次
    publish_hazard_lights_status();
    publish_turn_indicators_status();
    publish_gear_status(); 
    publish_control_mode();
    // ... 
}

void akm_interface::publish_steering_status()
{
    autoware_auto_vehicle_msgs::msg::SteeringReport msg;
    msg.stamp = this->now();
    // 如果里程计或IMU提供转向角，则使用；否则填充0
    // 例如从最新控制指令获取目标转向角（或从车辆反馈）
    if (latest_control_cmd_) {
        msg.steering_tire_angle = latest_control_cmd_->lateral.steering_tire_angle;
    } else {
        msg.steering_tire_angle = 0.0;
    }
    // 若您有实际的转向角反馈，请从 odom 或 imu 中提取
    steering_status_pub_->publish(msg);
}

void akm_interface::publish_gear_status()
{
    autoware_auto_vehicle_msgs::msg::GearReport msg;
    msg.stamp = this->now();
    // 默认档位为 DRIVE（若小车无档位概念）
    msg.report = autoware_auto_vehicle_msgs::msg::GearReport::DRIVE;
    // 如果接收到 gear_cmd 并已执行，可反馈执行后的档位
    // 根据 latest_gear_cmd_ 设置，若无则保持默认
    gear_status_pub_->publish(msg);
}

void akm_interface::publish_control_mode()
{
    autoware_auto_vehicle_msgs::msg::ControlModeReport msg;
    msg.stamp = this->now();
    // 固定为 AUTONOMOUS（若车辆支持手动/自动切换，可根据状态更改）
    msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    control_mode_pub_->publish(msg);
}

void akm_interface::publish_velocity_status()
{
    if (!latest_odom_) return;
    
    autoware_auto_vehicle_msgs::msg::VelocityReport msg;
    // msg.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.longitudinal_velocity = latest_odom_->twist.twist.linear.x;
    msg.lateral_velocity = latest_odom_->twist.twist.linear.y;
    msg.heading_rate = latest_odom_->twist.twist.angular.z;
    velocity_status_pub_->publish(msg);
}

void akm_interface::publish_battery_status()
{
    tier4_vehicle_msgs::msg::BatteryStatus msg;
    msg.stamp = this->now();
    msg.energy_level = 100.0; // 剩余容量百分比
    // 根据你的小车实际数据填充
    battery_pub_->publish(msg);
}

void akm_interface::publish_hazard_lights_status()
{
    autoware_auto_vehicle_msgs::msg::HazardLightsReport msg;
    msg.stamp = this->now();
    msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::DISABLE; // 默认关闭
    // 如果收到命令，则使用命令中的值
    hazard_lights_pub_->publish(msg);
}

void akm_interface::publish_turn_indicators_status()
{
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport msg;
    msg.stamp = this->now();
    msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE; // 默认无转向
    turn_indicators_pub_->publish(msg);
}

void akm_interface::callback_control_cmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
    latest_control_cmd_ = msg;
    control_timeout_ = false;  // 收到指令，重置超时标志
}

void akm_interface::callback_gear_cmd(
    const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
    latest_gear_cmd_ = msg;
    // 若小车支持档位控制，可在此处理；否则忽略
}


void akm_interface::callback_turn_indicators_cmd(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
    // 记录命令，并在下次发布状态时使用
    latest_turn_cmd_ = msg;
    // 如果有硬件控制，这里调用硬件接口
    // 然后发布状态反馈（在 publish_turn_indicators_status 中使用 msg->command）
}

void akm_interface::callback_hazard_lights_cmd(
    const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
    latest_hazard_cmd_ = msg;
    // 类似处理
}

void akm_interface::control_loop()
{
    if (!latest_control_cmd_ || control_timeout_) {
        // 无指令或超时，发送零速度
        geometry_msgs::msg::Twist zero_cmd;
        zero_cmd.linear.x = 0.0;
        zero_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(zero_cmd);
        return;
    }

    // 调用转换函数
    geometry_msgs::msg::Twist cmd_vel;
    control_command_to_vehicle(latest_control_cmd_, cmd_vel);
    cmd_vel_pub_->publish(cmd_vel);
}

void akm_interface::control_command_to_vehicle(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr cmd,
    geometry_msgs::msg::Twist & twist)
{
    // 纵向速度（单位 m/s）
    double speed = cmd->longitudinal.speed;

    // 前轮转角（弧度）
    double steer_angle = cmd->lateral.steering_tire_angle;

    // 将转向角转换为角速度：ω = v * tan(δ) / L
    // 若小车直接接收转向角（如舵机角度），则需修改为 angular.z = steer_angle（但 Twist 通常表示角速度）
    // 这里按照阿克曼模型转换
    double angular = speed * std::tan(steer_angle) / wheelbase_;

    twist.linear.x = speed;
    twist.angular.z = angular;

    // 注意：若速度可能为负（倒车），角速度符号需根据速度调整，上述公式适用
}


void akm_interface::check_timeout()
{
    // 如果超过1秒未收到 control_cmd，设置超时标志
    static auto last_time = this->now();
    auto now_time = this->now();
    if (latest_control_cmd_) {
        // 检查最新指令的时间戳
        auto cmd_time = latest_control_cmd_->stamp;
        if ((now_time - cmd_time).seconds() > 1.0) {
            control_timeout_ = true;
        }
    } else {
        control_timeout_ = true;
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