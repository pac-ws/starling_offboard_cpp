#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <GeographicLib/Geodesic.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iostream>
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
#include <std_msgs/msg/int32_multi_array.hpp>
#include <string>
#include <string_view>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class StarlingOffboard : public rclcpp::Node {
 public:
  enum class ControlMode {POS, VEL};
  enum class State { IDLE, PREFLT, ARMING, TAKEOFF, MISSION, LANDING };
  static std::string StateToString(StarlingOffboard::State state) {
    switch (state) {
      case StarlingOffboard::State::IDLE:
        return "IDLE";
      case StarlingOffboard::State::PREFLT:
        return "PREFLT";
      case StarlingOffboard::State::ARMING:
        return "ARMING";
      case StarlingOffboard::State::TAKEOFF:
        return "TAKEOFF";
      case StarlingOffboard::State::MISSION:
        return "MISSION";
      case StarlingOffboard::State::LANDING:
        return "LANDING";
      default:
        return "INVALID";
    }
  }

  StarlingOffboard();

 private:
  // Number of waypoints to set before attempting to enter offboard mode
  size_t offboard_setpoint_counter_ =
      0;  //!< counter for the number of setpoints sent
  nav_msgs::msg::Path path_;
  uint8_t arming_state_;

  // Mission Control
  bool enable_ = false;
  bool takeoff_ = false;
  bool land_ = false;
  std::shared_ptr<rclcpp::ParameterEventHandler> mission_control_PEH_ptr_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_enable_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_takeoff_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_land_;
  rclcpp::SyncParametersClient::SharedPtr sync_parameters_client_;

  bool gps_received_ = false;
  bool origin_gps_received_ = false;

  bool takeoff_completed_ = false;

  // GPS coordinates at the launch site
  double launch_gps_lat_;
  double launch_gps_lon_;

  // GPS coordinates + heading of the mission origin (GCS)
  double mission_origin_lon_;
  double mission_origin_lat_;
  double heading_;

  double yaw_;

  Eigen::Matrix<double, 4, 4> T_miss_ned_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 4> T_ned_miss_ = Eigen::Matrix4d::Identity();

  State state_ = State::IDLE;

  // Holds the current velocity from the mission to be sent to the px4
  Eigen::Vector4d vel_ned_ = Eigen::Vector4d::Unit(3);
  // Used to stop the drone when it reaches the waypoint
  Eigen::Vector4d stop_vel_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d takeoff_pos_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d takeoff_pos_ned_ = Eigen::Vector4d::Unit(3);
  Eigen::Vector4d curr_position_ = Eigen::Vector4d::Unit(3);

  rclcpp::Time time_last_vel_update_;

  // Timer drives the main loop
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr path_pub_timer_;
  rclcpp::Clock::SharedPtr clock_;

  px4_msgs::msg::VehicleLocalPosition pos_msg_;
  px4_msgs::msg::SensorGps gps_pos_msg_;

  rclcpp::QoS qos_;

  // Parameters
  struct Params {
    size_t buffer_size;
    int robot_id = 1;
    double position_tolerance = 1.;  // waypoint position tolerance in meters
    double env_scale_factor = 1.0;
    double max_speed = 2.0;
    double x_takeoff;
    double y_takeoff;
    double z_takeoff;
  };
  Params params_;

  struct Subscriptions {
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
        mission_origin_gps;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr launch_gps;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
        vehicle_status;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        vehicle_local_pos;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
        mission_control;
  };
  Subscriptions subs_;

  struct Publishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drone_status;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        traj_setpoint;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_control_mode;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command;
  };
  Publishers pubs_;

  inline double ConvertRawGPSToDegrees(const int32_t raw) {
    return (double)raw / 1e7;
  }

  void GetNodeParameters();
  void GetMissionControl();
  void GetMissionOriginGPS();
  void GetLaunchGPS();
  void InitializeSubscribers();
  void InitializePublishers();
  void ComputeTransforms();
  void Arm();
  void Disarm();
  void TimerCallback();
  void PathPublisherTimerCallback();
  bool HasReachedPos(const Eigen::Vector4d& target_pos);
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

  inline void ClampVelocity(Eigen::Vector4d& vel) noexcept {
    vel.x() = std::clamp(vel.x(), -params_.max_speed, params_.max_speed);
    vel.y() = std::clamp(vel.y(), -params_.max_speed, params_.max_speed);
    vel.z() = std::clamp(vel.z(), -params_.max_speed, params_.max_speed);
  }
  static std::string EigenMatToStr(const Eigen::Matrix<double, 4, 4>& mat) {
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::stringstream ss;
    ss << mat.format(CleanFmt);
    return ss.str();
  }

  /**
   * @brief Transform the position from mission frame to NED
   */
  Eigen::Vector4d TransformVec(const Eigen::Vector4d& vec,
                               const Eigen::Matrix<double, 4, 4>& Tf) {
    Eigen::Vector4d transformed_vec = Tf * vec;
    return transformed_vec;
  }
};

std::ostream& operator<<(std::ostream& os, StarlingOffboard::State state) {
  return os << StarlingOffboard::StateToString(state);
}
