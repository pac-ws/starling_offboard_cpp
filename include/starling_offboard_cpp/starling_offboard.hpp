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
#include <px4_msgs/msg/vehicle_attitude.hpp>
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
using namespace px4_msgs::msg;

class StarlingOffboard : public rclcpp::Node {
 public:
  enum class ControlMode { POS, VEL };
  StarlingOffboard();

 private:
  // --------------- PX4 Offboard --------------- 
  // Number of waypoints to set before attempting to enter offboard mode
  size_t offboard_setpoint_counter_ = 0;
  uint8_t arming_state_ = 0;

  // --------------- Service Clients --------------- 
  std::shared_ptr<ServiceClient<rcl_interfaces::srv::GetParameters>> launch_gps_sc_;
  std::shared_ptr<ServiceClient<rcl_interfaces::srv::GetParameters>> mission_origin_gps_sc_;
  std::shared_ptr<ServiceClient<async_pac_gnn_interfaces::srv::SystemInfo>> system_info_sc_;
  
  // --------------- Mission Control Flags--------------- 
  async_pac_gnn_interfaces::msg::MissionControl mission_control_;
  bool ob_enable_ = false;
  bool ob_takeoff_ = false;
  bool ob_land_ = false;
  bool geofence_ = false;

  // --------------- State Flags --------------- 
  // bool origin_gps_received_ = false;
  bool mission_control_received_ = false;
  bool ned_cam_computed_ = false;
  bool takeoff_completed_ = false;
  bool reached_land_pos_h_ = false;
  bool reached_land_pos_v_ = false;
  bool reached_land_stationary_v_ = false;
  bool pos_msg_received_ = false;
  bool att_msg_received_ = false;
  bool sent_p_ned = false;
  bool breach_ = false;

  // --------------- Descent --------------- 
  bool descent_started_ = false;
  double descent_time_ = 0.0;
  rclcpp::Time descent_start_time_;

  // --------------- Launch Coords --------------- 
  double launch_gps_lat_;
  double launch_gps_lon_;

  // --------------- GCS Origin Coords --------------- 
  double mission_origin_lon_;
  double mission_origin_lat_;
  double heading_;

  double azimuth_ = 0.0;
  double yaw_;
  double alt_offset_;

  // --------------- Transforms --------------- 
  Eigen::Matrix<double, 4, 4> T_miss_ned_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 4> T_ned_miss_ = Eigen::Matrix4d::Identity();

  // --------------- State Machine --------------- 
  State state_ = State::INIT_START;

  // --------------- Key Actions/Configurations --------------- 
  // Holds the current velocity from the mission to be sent to the px4
  Eigen::Vector4d vel_ned_ = Eigen::Vector4d::Unit(3);
  // Used to stop the drone when it reaches the waypoint
  Eigen::Vector4d stop_vel_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d land_vel_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d start_pos_ned_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d takeoff_pos_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d takeoff_pos_ned_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d landing_pos_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d landing_pos_ned_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d curr_position_ = Eigen::Vector4d::Unit(3);

  // --------------- Mission Timeout --------------- 
  rclcpp::Time time_last_vel_update_;

  // --------------- Key Timers --------------- 
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;

  // --------------- PX4 messages --------------- 
  px4_msgs::msg::VehicleAttitude att_msg_;
  px4_msgs::msg::VehicleLocalPosition pos_msg_;
  px4_msgs::msg::SensorGps gps_pos_msg_;
  px4_msgs::msg::VehicleGlobalPosition global_pos_msg_;
  geometry_msgs::msg::PoseStamped gnn_pose_;

  // --------------- ROS QOS --------------- 
  rclcpp::QoS qos_;

  // --------------- Geofence ---------------
  double fence_x_min_;
  double fence_x_max_;
  double fence_y_min_;
  double fence_y_max_;
  double fence_recovery_x_min_;
  double fence_recovery_x_max_;
  double fence_recovery_y_min_;
  double fence_recovery_y_max_;
  bool geofence_is_set_ = false;

  // --------------- Scaling --------------- 
  double env_scale_factor_;
  double vel_scale_factor_;

  // --------------- Helper Structs --------------- 
  Intervals intervals_;
  Params params_;
  Subscriptions subs_;
  Publishers pubs_;

  inline double ConvertRawGPSToDegrees(const int32_t raw) {
    return (double)raw / 1e7;
  }

  // --------------- Prototypes --------------- 
  void GetNodeParameters();
  void InitializeGeofence();
  void GetSystemInfo();
  void GetLaunchGPS();
  void GetMissionOriginGPS();
  void InitializeSubscribers();
  void InitializePublishers();
  void ComputeLocalMissionTransform();
  void ComputeStartPosTakeoff();
  void ComputeTagNedTransform();
  void ComputeLocalBodyTransform();
  void ComputeTagCamTransform();
  void ComputeTagLocalTransform();
  void ComputeTagBodyTransform();
  void ComputeExtrinsicTransforms();
  void Arm();
  void Disarm();
  void GeofenceCheck();
  void TimerCallback();
  void StatusTimerCallback();
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
  void PubTransforms();

  /**
   * @brief Transform the position from mission frame to NED
   */
};
}  // namespace pac_ws::starling_offboard
