#pragma once

namespace pac_ws::starling_offboard {

struct Intervals {
  std::chrono::milliseconds Short{30};
  std::chrono::milliseconds Mid{100};
  std::chrono::milliseconds Long{1000};
  std::chrono::milliseconds VLong{2000};
};

struct Params {
  size_t buffer_size;
  int robot_id = 1;
  double position_tolerance = 1.;  // waypoint position tolerance in meters
  double env_scale_factor = 1.0;
  double max_speed = 2.0;
  double kP = 1.0;
  double x_takeoff;
  double y_takeoff;
  double z_takeoff;
  double land_vel_z;
  double world_size = 1024.0;
  double fence_x_buf_l = 10.0;
  double fence_x_buf_r = 10.0;
  double fence_y_buf_b = 10.0;
  double fence_y_buf_t = 10.0;
  double kP_land = 1.0;
  double kD_land = 0.1;
};

struct Subscriptions {
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel;
  // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr mission_origin_gps;
  // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr launch_gps;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      vehicle_local_pos;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr
      vehicle_global_pos;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr vehicle_gps_pos;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_tag_cam;
  rclcpp::Subscription<async_pac_gnn_interfaces::msg::MissionControl>::SharedPtr
      mission_control;
};

struct Publishers {
  rclcpp::Publisher<async_pac_gnn_interfaces::msg::RobotStatus>::SharedPtr
      status;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_control_mode;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command;
};
}  // namespace pac_ws::starling_offboard
