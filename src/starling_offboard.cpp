#include "starling_offboard_cpp/starling_offboard.hpp"
#include <istream>

namespace pac_ws::starling_offboard {
StarlingOffboard::StarlingOffboard() : Node("starling_offboard"), qos_(1) {
  clock_ = std::make_shared<rclcpp::Clock>();
  time_last_vel_update_ = clock_->now();
  
  GetNodeParameters();
  
  // QoS
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_ = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, params_.buffer_size),
      qos_profile);

  land_vel_[2] = params_.land_vel_z;

  // z_takeoff is the altitude set by the user
  // alt_offset_'s value is produced by homify to account for altitude offsets on the ground.
  // RCLCPP_INFO(this->get_logger(), "Takeoff position: %f, %f, %f", params_.x_takeoff, params_.y_takeoff, params_.z_takeoff + alt_offset_);
  // takeoff_pos_ << params_.x_takeoff, params_.y_takeoff, params_.z_takeoff + alt_offset_, 1.0;

  GetLaunchGPS();
  GetMissionOriginGPS();
  GetSystemInfo();

  timer_ = this->create_wall_timer(
      intervals_.Mid, std::bind(&StarlingOffboard::TimerCallback, this));

  RCLCPP_WARN(this->get_logger(), "Initialized Timers");
}

void StarlingOffboard::GetNodeParameters() {
  this->declare_parameter("buffer_size", 5);
  this->get_parameter("buffer_size", params_.buffer_size);

  this->declare_parameter<int>("robot_id", 1);
  this->get_parameter("robot_id", params_.robot_id);

  this->declare_parameter<double>("position_tolerance", 1.0);
  this->get_parameter("position_tolerance", params_.position_tolerance);

  this->declare_parameter<double>("env_scale_factor", 50.0);
  this->get_parameter("env_scale_factor", params_.env_scale_factor);
  // Default to the parameter value if not set by the system info service
  env_scale_factor_ = params_.env_scale_factor;

  this->declare_parameter<double>("x_takeoff", 0.0);
  this->get_parameter("x_takeoff", params_.x_takeoff);

  this->declare_parameter<double>("y_takeoff", 0.0);
  this->get_parameter("y_takeoff", params_.y_takeoff);

  this->declare_parameter<double>("alt", 2.0);
  this->get_parameter("alt", params_.z_takeoff);

  this->declare_parameter<double>("max_speed", 2.0);
  this->get_parameter("max_speed", params_.max_speed);

  this->declare_parameter<double>("kP", 1.0);
  this->get_parameter("kP", params_.kP);

  this->declare_parameter<double>("world_size", 1024.0);
  this->get_parameter("world_size", params_.world_size);

  this->declare_parameter<double>("land_vel_z", 0.5);
  this->get_parameter("land_vel_z", params_.land_vel_z);

  this->declare_parameter<double>("fence_x_buf_l", 10.0);
  this->get_parameter("fence_x_buf_l", params_.fence_x_buf_l);

  this->declare_parameter<double>("fence_x_buf_r", 10.0);
  this->get_parameter("fence_x_buf_r", params_.fence_x_buf_r);

  this->declare_parameter<double>("fence_y_buf_b", 10.0);
  this->get_parameter("fence_y_buf_b", params_.fence_y_buf_b);

  this->declare_parameter<double>("fence_y_buf_t", 10.0);
  this->get_parameter("fence_y_buf_t", params_.fence_y_buf_t);
}

void StarlingOffboard::InitializeGeofence(){
  fence_x_min_ =  -params_.fence_x_buf_l;
  fence_x_max_ = params_.world_size / env_scale_factor_ + params_.fence_x_buf_r;
  fence_y_min_ = -params_.fence_y_buf_b;
  fence_y_max_ = params_.world_size / env_scale_factor_ + params_.fence_y_buf_t;
  RCLCPP_INFO(this->get_logger(), "Geofence initialized:");
  RCLCPP_INFO(this->get_logger(), "Dimensions (L, R, B, T): %f, %f, %f, %f", fence_x_min_, fence_x_max_, fence_y_min_, fence_y_max_);
}

void StarlingOffboard::InitializeSubscribers() {
  subs_.vehicle_status =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(
          "fmu/out/vehicle_status", qos_,
          [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
            arming_state_ = msg->arming_state;
          });

  // Velocity Translation (TwistStamped [GNN] to TrajectorySetpoint [PX4])
  subs_.cmd_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", qos_,
      std::bind(&StarlingOffboard::UpdateVel, this, std::placeholders::_1));

  // Position Translation (VehicleLocalPosition [PX4] to PoseStamped [GNN])
  subs_.vehicle_local_pos =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "fmu/out/vehicle_local_position", qos_,
          std::bind(&StarlingOffboard::VehicleLocalPosCallback, this,
                    std::placeholders::_1));
  subs_.vehicle_global_pos =
      this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
          "fmu/out/vehicle_global_position", qos_,
              [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                global_pos_msg_ = *msg;
              });
  subs_.vehicle_gps_pos =
      this->create_subscription<px4_msgs::msg::SensorGps>(
          "fmu/out/vehicle_gps_position", qos_,
              [this](const px4_msgs::msg::SensorGps::SharedPtr msg) {
                gps_pos_msg_ = *msg;
              });
  subs_.vehicle_attitude =
      this->create_subscription<px4_msgs::msg::VehicleAttitude>(
          "fmu/out/vehicle_attitude", qos_,
              [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                att_msg_ = *msg;
                att_msg_received_ = true;
              });
  subs_.tf_tag_cam =
      this->create_subscription<tf2_msgs::msg::TFMessage>(
          "/tf", qos_,
              [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                tf_tag_cam_msg_ = *msg;
                RCLCPP_INFO_ONCE(this->get_logger(), "Transform received");
                if (!tf_tag_cam_msg_.transforms.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Tag detected");
                    tf_tag_cam_received_ = true;
                }
              });
  // subs_.mission_origin_gps = 
  //     this->create_subscription<geometry_msgs::msg::Point>(
  //             "/mission_origin_gps", qos_,
  //             [this](const geometry_msgs::msg::Point::SharedPtr msg) {
  //               mission_origin_lon_ = msg->x;
  //               mission_origin_lat_ = msg->y;
  //               heading_ = msg->z;
  //               origin_gps_received_ = true;
  //             });
  subs_.mission_control = 
      this->create_subscription<async_pac_gnn_interfaces::msg::MissionControl>(
              "/mission_control", 10,
              [this](const async_pac_gnn_interfaces::msg::MissionControl::SharedPtr msg) {
                ob_enable_ = msg->ob_enable;
                ob_takeoff_ = msg->ob_takeoff;
                ob_land_ = msg->ob_land;
                geofence_ = msg->geofence;
                mission_control_received_ = true;
              });
}

void StarlingOffboard::InitializePublishers() {
  pubs_.pose =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", qos_);

  auto qos_reliable = rclcpp::QoS(rclcpp::QoSInitialization(
      rmw_qos_profile_default.history, params_.buffer_size));
  qos_reliable.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  pubs_.status =
      this->create_publisher<async_pac_gnn_interfaces::msg::RobotStatus>("status", qos_);

  pubs_.offboard_control_mode =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          "fmu/in/offboard_control_mode", params_.buffer_size);
  pubs_.vehicle_command = this->create_publisher<VehicleCommand>(
      "fmu/in/vehicle_command", params_.buffer_size);
  pubs_.traj_setpoint =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
          "fmu/in/trajectory_setpoint", params_.buffer_size);
  status_timer_ = this->create_wall_timer(
      intervals_.Long, std::bind(&StarlingOffboard::StatusTimerCallback, this));
}

/**
 * @brief Main Loop
 */
void StarlingOffboard::TimerCallback() {
  if (att_msg_received_ && pos_msg_received_){
      ComputeLocalBodyTransform();
  }

  // Geofence check
  if(geofence_) {
    if (geofence_is_set_) {
      GeofenceCheck();
    } else {
      if (system_info_sc_->Done()) {
        InitializeGeofence();
        geofence_is_set_ = true;
      }
    }
  }

  // State Machine
  switch (state_) {
    case State::INIT_START: {
      InitializeSubscribers();
      InitializePublishers();
      RCLCPP_WARN(this->get_logger(), "Initialized pubs and subs");
      state_ = State::INIT_GPS;
      break;
    }

    case State::INIT_GPS: {
      if (launch_gps_sc_->Done()) {
        RCLCPP_INFO(this->get_logger(), "Launch GPS received");
        state_ = State::IDLE;
      }
      break;
    }

    case State::IDLE: {
      if (!mission_control_received_) {
          RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for mission control topic...");
          break;
      }
      RCLCPP_INFO_ONCE(this->get_logger(), "Mission control received.");

      if (!mission_origin_gps_sc_->Done()) {
          RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for mission origin GPS...");
          break;
      }
      RCLCPP_INFO_ONCE(this->get_logger(), "Mission origin GPS received.");

      // Global origin
      RCLCPP_DEBUG(this->get_logger(), "Global origin");
      RCLCPP_DEBUG(this->get_logger(), "lat: %.8f", mission_origin_lat_);
      RCLCPP_DEBUG(this->get_logger(), "lon: %.8f", mission_origin_lon_);
      RCLCPP_DEBUG(this->get_logger(), "heading: %.8f", heading_);

      // Global startup location
      RCLCPP_DEBUG(this->get_logger(), "Global launch received");
      RCLCPP_DEBUG(this->get_logger(), "lat: %.8f", launch_gps_lat_);
      RCLCPP_DEBUG(this->get_logger(), "lon: %.8f", launch_gps_lon_);

      ComputeLocalMissionTransform();
      ComputeExtrinsicTransforms();
      ComputeStartPosTakeoff();

      RCLCPP_WARN_ONCE(this->get_logger(), "kP: %f", params_.kP);

      state_ = State::PREFLT;
      RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
      break;
    }

    case State::PREFLT:
      RCLCPP_INFO_ONCE(this->get_logger(), "Received tag_wrt_cam tf? %d", tf_tag_cam_received_);
      if (ob_takeoff_ && ob_enable_) {
        if (geofence_ && !geofence_is_set_) {
          RCLCPP_WARN(this->get_logger(), "Geofence is requested but not set. Will not arm. Is get_system_info service available?");
        }
        state_ = State::ARMING;
        RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
        RCLCPP_DEBUG(this->get_logger(), "yaw_: %f", yaw_);
      } 
      break;

    case State::ARMING:
      if (offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        this->PubVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        this->Arm();
        if (arming_state_ == 2) {
          RCLCPP_INFO(this->get_logger(), "Vehicle armed");
          state_ = State::TAKEOFF;
          RCLCPP_DEBUG(this->get_logger(), "yaw_: %f", yaw_);
          RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "Vehicle not armed");
          // Retry
          offboard_setpoint_counter_ = 0;
        }
      }
      // Send 10 setpoints before attempting to Arm
      // offboard_control_mode needs to be paired with trajectory_setpoint
      PubOffboardControlMode(ControlMode::POS);
      PubTrajSetpointPos(takeoff_pos_ned_);
      if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
      }
      break;
    

    case State::TAKEOFF:
      takeoff_completed_ = 
        HasReachedPos(pos_msg_, takeoff_pos_ned_, params_.position_tolerance);
      if (takeoff_completed_) {
        PubOffboardControlMode(ControlMode::VEL);
        PubTrajSetpointVel(stop_vel_);
        state_ = State::MISSION;
        RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Takeoff complete -- reached setpoint within TOL");
      } else {
        PubOffboardControlMode(ControlMode::POS);
        PubTrajSetpointPos(takeoff_pos_ned_);
      }
      break;
    
    case State::MISSION: {
      if (ob_land_) {
        state_ = State::LANDING_H;
        RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
        RCLCPP_WARN_ONCE(this->get_logger(), "Landing sequence initiated");
        break;
      }

      // offboard_control_mode needs to be paired with trajectory_setpoint
      // If the message rate drops bellow 2Hz, the drone exits offboard control
      PubOffboardControlMode(ControlMode::VEL);
      rclcpp::Time time_now = clock_->now();
      rclcpp::Duration duration = time_now - time_last_vel_update_;
      if (duration.seconds() > 1.0) {
        double err_z = (pos_msg_.z + params_.z_takeoff);
      stop_vel_[2] = -1.0 * err_z;
        ClampVelocity(params_.max_speed, stop_vel_);
        PubTrajSetpointVel(stop_vel_);
        //RCLCPP_INFO(this->get_logger(), "Mission velocity update timeout; stop velocity (%f, %f, %f)", stop_vel_[0], stop_vel_[1], stop_vel_[2]);
      }
      else {
        PubTrajSetpointVel(vel_ned_);
      }
      break;
    }

    case State::LANDING_H:
      reached_land_pos_h_ = 
        HasReachedPos(pos_msg_, takeoff_pos_ned_, params_.position_tolerance);
      if (reached_land_pos_h_) {
        state_ = State::LANDING_V;
        RCLCPP_WARN_ONCE(this->get_logger(), "Landing sequence initiated");
        RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
      }
      else{
        // Lateral move to the takeoff position
        PubOffboardControlMode(ControlMode::POS);
        PubTrajSetpointPos(takeoff_pos_ned_);
      }
      break;

    case State::LANDING_V:
      ComputeTagCamTransform();
      ComputeTagLocalTransform();
      ComputeTagBodyTransform();

      reached_land_pos_v_ =
        HasReachedPos(pos_msg_, start_pos_ned_, params_.position_tolerance);
      reached_land_stationary_v_ = std::abs(pos_msg_.vz) < 0.1;
      if (reached_land_pos_v_ && reached_land_stationary_v_) {
        state_ =  State::DISARM;
        RCLCPP_WARN_ONCE(this->get_logger(), "Landing sequence finished");
        RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
      }
      else{
          if (tf_tag_cam_received_) {
            yaw_ = -std::atan2(T_tag_body_(1, 0), T_tag_body_(0, 0)); // Assumption, only correcting yaw
            const Eigen::Vector4d p_tag(0,0,0,1); // The position of the tag in the fixed frame.
            Eigen::Vector4d p_ned = TransformVec(p_tag, T_tag_ned_);
            RCLCPP_INFO(this->get_logger(), "p_ned: %s" , EigenToStr(p_ned).c_str());
            PubOffboardControlMode(ControlMode::POS);
            PubTrajSetpointPos(p_ned);
          }
          else {
            PubOffboardControlMode(ControlMode::VEL);
            PubTrajSetpointVel(land_vel_);
          }
      }
      break;

    case State::DISARM:
      if (arming_state_ == 2) {
        this->Disarm();
      }
      else{
          RCLCPP_WARN_ONCE(this->get_logger(), "Vehicle disarmed");
      }
      // Need to restart the drone after landing so just spin in this state.
      break;
  }
}

void StarlingOffboard::StatusTimerCallback() {

  auto status_msg = async_pac_gnn_interfaces::msg::RobotStatus();
  status_msg.batt = 100; // TODO
  status_msg.state = StateToString(state_);
  status_msg.breach = breach_;
  status_msg.gps_lat = global_pos_msg_.lat;
  status_msg.gps_lon = global_pos_msg_.lon;
  status_msg.gps_alt = global_pos_msg_.alt;
  status_msg.gps_sats = gps_pos_msg_.satellites_used;
  status_msg.gps_heading = gps_pos_msg_.heading;
  status_msg.local_pos_x = pos_msg_.x;
  status_msg.local_pos_y = pos_msg_.y;
  status_msg.local_pos_z = pos_msg_.z;
  status_msg.local_pos_heading = pos_msg_.heading;
  status_msg.pose_x = gnn_pose_.pose.position.x;
  status_msg.pose_y = gnn_pose_.pose.position.y;
  status_msg.pose_z = gnn_pose_.pose.position.z;
  status_msg.ned_vel_x = vel_ned_[0];
  status_msg.ned_vel_y = vel_ned_[1];
  status_msg.ned_vel_z = vel_ned_[2];
  pubs_.status->publish(status_msg);
}

void StarlingOffboard::PathPublisherTimerCallback() {
  // Publish the path
  path_.header.stamp = this->get_clock()->now();
  path_.header.frame_id = "map";

  if (path_.poses.size() == 0) {
    return;
  }
  if (std::abs(curr_position_[0]) < 0.01 &&
      std::abs(curr_position_[1]) < 0.01) {
    return;
  }
  Eigen::Vector4d scaled_pos = curr_position_ * env_scale_factor_;
  geometry_msgs::msg::PoseStamped latest_pose = path_.poses.back();
  double max_dist =
      std::max(std::abs(latest_pose.pose.position.x - scaled_pos[0]),
               std::abs(latest_pose.pose.position.y - scaled_pos[1]));
  max_dist =
      std::max(max_dist, std::abs(latest_pose.pose.position.z - scaled_pos[2]));
  if (max_dist < 1) {
    pubs_.nav_path->publish(path_);
    return;
  }
  latest_pose.header.stamp = this->get_clock()->now();
  latest_pose.header.frame_id = "map";

  latest_pose.pose.position.x = scaled_pos[0];
  latest_pose.pose.position.y = scaled_pos[1];
  latest_pose.pose.position.z = scaled_pos[2];
  path_.poses.push_back(latest_pose);
  pubs_.nav_path->publish(path_);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void StarlingOffboard::Arm() {
  PubVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0,
                    0.0, 0.0, 0.0);
  RCLCPP_INFO_ONCE(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void StarlingOffboard::Disarm() {
  PubVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0,
                    0.0, 0.0, 0.0);
  RCLCPP_INFO_ONCE(this->get_logger(), "Disarm command send");
}

/**
 * @brief Check for a geofence violation
 */
void StarlingOffboard::GeofenceCheck() {
  if (geofence_) {
    const Eigen::Vector4d pos_vec(pos_msg_.x, pos_msg_.y, pos_msg_.z, 1.0);
    const Eigen::Vector4d vehicle_mission_position =
        TransformVec(pos_vec, T_ned_miss_);

    if (vehicle_mission_position[0] < fence_x_min_ ||
        vehicle_mission_position[0] > fence_x_max_ ||
        vehicle_mission_position[1] < fence_y_min_ ||
        vehicle_mission_position[1] > fence_y_max_) {
        
      // Warn on the first breach until breach is reset
      if (!breach_) {
        RCLCPP_WARN(this->get_logger(), "Geofence breach detected.");
      }
      breach_ = true;
    }
    else{
        breach_ = false;
    }
  }
  // In case Geofence is turned off mid-flight
  else{
    breach_ = false;
  }
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void StarlingOffboard::PubOffboardControlMode(const StarlingOffboard::ControlMode mode) {
  px4_msgs::msg::OffboardControlMode msg{};

  if (mode == ControlMode::POS) {
    msg.position = true;
    msg.velocity = false;
  } else if (mode == ControlMode::VEL) {
    msg.position = false;
    msg.velocity = true;
  } else {
    msg.position = false;
    msg.velocity = false;
  }
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  if (ob_enable_) {
    pubs_.offboard_control_mode->publish(msg);
  }
}

/**
 * @brief Publish a trajectory setpoint (vel)
 */
void StarlingOffboard::PubTrajSetpointVel(const Eigen::Vector4d& target_vel) {
  TrajectorySetpoint msg{};
  msg.position = {std::nanf(""), std::nanf(""),
                  std::nanf("")};  // required for vel control in px4
  if (ob_enable_ && !breach_) {
      msg.velocity = {static_cast<float>(target_vel[0]),
                      static_cast<float>(target_vel[1]),
                      static_cast<float>(target_vel[2])
      };
  }
  // Hold position if not enabled or breach
  else {
      msg.velocity = {static_cast<float>(0.0),
                      static_cast<float>(0.0),
                      static_cast<float>(0.0)
      };
  }
  msg.yaw = static_cast<float>(yaw_);  // [-PI:PI]
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  pubs_.traj_setpoint->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint (position)
 */
void StarlingOffboard::PubTrajSetpointPos(const Eigen::Vector4d& target_pos) {
  TrajectorySetpoint msg{};
  msg.position = {static_cast<float>(target_pos[0]),
                  static_cast<float>(target_pos[1]),
                  static_cast<float>(target_pos[2])};
  msg.yaw = static_cast<float>(yaw_);  // [-PI:PI]
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  if (ob_enable_) {
    pubs_.traj_setpoint->publish(msg);
  }
}

/**
 * @brief Publish the trajectory setpoint (TwistStamped) to the PX4
 */
void StarlingOffboard::UpdateVel(
  const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel) {
  const double err_z = (params_.z_takeoff + pos_msg_.z);
  const Eigen::Vector4d vel_mission(cmd_vel->twist.linear.x,
                                    cmd_vel->twist.linear.y, 
                                    params_.kP * err_z, 
                                    0.0);
  vel_ned_ = TransformVec(vel_mission, T_miss_ned_);
  ClampVelocity(params_.max_speed, vel_ned_);
  time_last_vel_update_ = clock_->now();
}

/**
 * @brief Publish the pose (PoseStamped) to the GNN. Publishing the path as well
 * for visualization
 */
void StarlingOffboard::VehicleLocalPosCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr pos_msg) {
  pos_msg_received_ = true;
  // Only publish the pose once the drone is armed and transforms have been set
  azimuth_ = pos_msg->heading;
  if (state_ >= State::PREFLT) {
    pos_msg_ = *pos_msg;
    const Eigen::Vector4d pos_vec(pos_msg->x, pos_msg->y, pos_msg->z, 1.0);

    const Eigen::Vector4d vehicle_mission_position =
        TransformVec(pos_vec, T_ned_miss_);
    curr_position_ = vehicle_mission_position;
    if (path_.poses.size() == 0 && false) {
      if (std::abs(curr_position_[0]) < 1. &&
          std::abs(curr_position_[1]) < 1.) {
        return;
      }
      if (curr_position_[0] != 0 && curr_position_[1] != 0 &&
          curr_position_[2] != 0) {
        //geometry_msgs::msg::PoseStamped gnn_pose;
        gnn_pose_.header.stamp = this->get_clock()->now();
        gnn_pose_.header.frame_id = "map";
        gnn_pose_.pose.position.x = vehicle_mission_position[0];
        gnn_pose_.pose.position.y = vehicle_mission_position[1];
        gnn_pose_.pose.position.z = vehicle_mission_position[2];
        path_.poses.push_back(gnn_pose_);
      }
    }

    // Publish the current pose
    //geometry_msgs::msg::PoseStamped gnn_pose_;
    gnn_pose_.header.stamp = this->get_clock()->now();
    gnn_pose_.header.frame_id = "map";

    // TODO
    gnn_pose_.pose.position.x = vehicle_mission_position[0];
    gnn_pose_.pose.position.y = vehicle_mission_position[1];
    gnn_pose_.pose.position.z = vehicle_mission_position[2];
    pubs_.pose->publish(gnn_pose_);
  }
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD
 * codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * @param param5    Command parameter 5
 * @param param6    Command parameter 6
 * @param param7    Command parameter 7
 */
void StarlingOffboard::PubVehicleCommand(uint32_t command, double param1,
                                         double param2, double param5,
                                         double param6, double param7) {
  VehicleCommand msg{};
  msg.param1 = static_cast<float>(param1);
  msg.param2 = static_cast<float>(param2);
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = static_cast<float>(param7);
  msg.command = command;
  msg.target_system = static_cast<uint8_t>(params_.robot_id + 1);
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp =
      static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  pubs_.vehicle_command->publish(msg);
}
}  // namespace pac_ws::starling_offboard

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pac_ws::starling_offboard::StarlingOffboard>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
