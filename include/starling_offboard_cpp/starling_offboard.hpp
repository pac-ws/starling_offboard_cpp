#pragma once

#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <async_pac_gnn_interfaces/msg/robot_status.hpp>
#include <async_pac_gnn_interfaces/srv/system_info.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <async_pac_gnn_interfaces/msg/mission_control.hpp>
#include <string>
#include <string_view>
#include "state_enum.hpp"
#include "helper_structs.hpp"
#include "utils.hpp"

namespace pac_ws::starling_offboard {
// using namespace std::chrono;
// using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class StarlingOffboard : public rclcpp::Node {
 public:
  enum class ControlMode { POS, VEL };
  StarlingOffboard();

 private:
  // Number of waypoints to set before attempting to enter offboard mode
  size_t offboard_setpoint_counter_ = 0;
  nav_msgs::msg::Path path_;
  uint8_t arming_state_;

  // Mission Control
  async_pac_gnn_interfaces::msg::MissionControl mission_control_;
  bool ob_enable_ = false;
  bool ob_takeoff_ = false;
  bool ob_land_ = false;
  bool geofence_ = false;

  bool breach_ = false;

  std::shared_ptr<ServiceClient<rcl_interfaces::srv::GetParameters>> launch_gps_sc_;
  std::shared_ptr<ServiceClient<rcl_interfaces::srv::GetParameters>> mission_origin_gps_sc_;
  std::shared_ptr<ServiceClient<async_pac_gnn_interfaces::srv::SystemInfo>> system_info_sc_;

  // bool origin_gps_received_ = false;
  bool mission_control_received_ = false;
  bool tf_tag_cam_received_ = false;
  bool takeoff_completed_ = false;
  bool reached_land_pos_h_ = false;
  bool reached_land_pos_v_ = false;
  bool reached_land_stationary_v_ = false;

  // GPS coordinates at the launch site
  double launch_gps_lat_;
  double launch_gps_lon_;

  // GPS coordinates + heading of the mission origin (GCS)
  double mission_origin_lon_;
  double mission_origin_lat_;
  double heading_;

  double yaw_;
  double alt_offset_;

  Eigen::Matrix<double, 4, 4> T_miss_ned_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 4> T_ned_miss_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 4> T_ned_tag_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 4> T_tag_ned_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 4> T_tag_cam_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 4> T_cam_tag_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 4> T_cam_ned_ = Eigen::Matrix4d::Identity();

  State state_ = State::INIT_START;

  // Holds the current velocity from the mission to be sent to the px4
  Eigen::Vector4d vel_ned_ = Eigen::Vector4d::Unit(3);
  // Used to stop the drone when it reaches the waypoint
  Eigen::Vector4d stop_vel_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d land_vel_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d start_pos_ned_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d takeoff_pos_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d takeoff_pos_ned_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d curr_position_ = Eigen::Vector4d::Unit(3);

  rclcpp::Time time_last_vel_update_;

  // Timer drives the main loop
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr path_pub_timer_;
  rclcpp::Clock::SharedPtr clock_;

  px4_msgs::msg::VehicleLocalPosition pos_msg_;
  px4_msgs::msg::SensorGps gps_pos_msg_;
  px4_msgs::msg::VehicleGlobalPosition global_pos_msg_;
  geometry_msgs::msg::PoseStamped gnn_pose_;
  tf2_msgs::msg::TFMessage tf_tag_cam_msg_;

  rclcpp::QoS qos_;

  double fence_x_min_;
  double fence_x_max_;
  double fence_y_min_;
  double fence_y_max_;
  bool geofence_is_set_ = false;

  double env_scale_factor_;
  double vel_scale_factor_;

  Intervals intervals_;
  Params params_;
  Subscriptions subs_;
  Publishers pubs_;

  inline double ConvertRawGPSToDegrees(const int32_t raw) {
    return (double)raw / 1e7;
  }

  void GetNodeParameters();
  void InitializeGeofence();
  void GetSystemInfo();
  void GetLaunchGPS();
  void GetMissionOriginGPS();
  void InitializeSubscribers();
  void InitializePublishers();
  void ComputeTransforms();
  void ComputeStartPosTakeoff();
  void ComputeTagNedTransform();
  void ComputeNedCamTransform();
  void Arm();
  void Disarm();
  void GeofenceCheck();
  void TimerCallback();
  void StatusTimerCallback();
  void PathPublisherTimerCallback();
  Eigen::Vector4d ComputeVel(const Eigen::Vector4d& target_pos);
  void VehicleLocalPosCallback(
      const px4_msgs::msg::VehicleLocalPosition::SharedPtr);
  void PubVehicleCommand(uint32_t command, double param1 = 0.0,
                         double param2 = 0.0, double param5 = 0.0,
                         double param6 = 0.0, double param7 = 0.0);

  void UpdateVel(const geometry_msgs::msg::TwistStamped::SharedPtr);

  void PubOffboardControlMode(const ControlMode mode);
  void PubTrajSetpointVel(const Eigen::Vector4d& target_vel);
  void PubTrajSetpointPos(const Eigen::Vector4d& target_pos);

  /**
   * @brief Transform the position from mission frame to NED
   */
};
}  // namespace pac_ws::starling_offboard
