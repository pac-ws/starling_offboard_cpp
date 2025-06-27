#include "starling_offboard_cpp/starling_offboard.hpp"

StarlingOffboard::StarlingOffboard() : Node("starling_offboard"), qos_(1) {
  clock_ = std::make_shared<rclcpp::Clock>();
  time_last_vel_update_ = clock_->now();
  
  GetNodeParameters();
  
  // QoS
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_ = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, params_.buffer_size),
      qos_profile);

  // z_takeoff is the altitude set by the user
  // alt_offset_'s value is produced by homify to account for altitude offsets on the ground.
  RCLCPP_INFO(this->get_logger(), "Takeoff position: %f, %f, %f", params_.x_takeoff, params_.y_takeoff, params_.z_takeoff + alt_offset_);
  takeoff_pos_ << params_.x_takeoff, params_.y_takeoff, params_.z_takeoff + alt_offset_, 1.0;
  land_vel_[2] = params_.land_vel_z;

  std::string launch_service_name = "gps_fix/get_parameters";
  homify_parameters_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(launch_service_name);

  std::string sys_info_service_name ="/sim/get_system_info";
  sys_info_client_ = this->create_client<async_pac_gnn_interfaces::srv::SystemInfo>(sys_info_service_name);

  timer_ = this->create_wall_timer(
      100ms, std::bind(&StarlingOffboard::TimerCallback, this));


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

void StarlingOffboard::GetMissionControl(){
  sync_parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/mission_control");
  while (!sync_parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  
  auto result = sync_parameters_client_->get_parameters({"offboard_enable", "takeoff", "land", "geofence"});
  RCLCPP_DEBUG(this->get_logger(), "result size = %ld", result.size());
  for (auto &p : result) {
    if (p.get_name() == "offboard_enable") {
      offboard_enable_ = p.get_value<bool>();
      RCLCPP_INFO(this->get_logger(), "init offboard_enable = %d", offboard_enable_);
    }
    if (p.get_name() == "takeoff") {
      takeoff_ = p.get_value<bool>();
      RCLCPP_INFO(this->get_logger(), "init takeoff = %d", takeoff_);
    }
    if (p.get_name() == "land") {
      land_ = p.get_value<bool>();
      RCLCPP_INFO(this->get_logger(), "init land = %d", land_);
    }
    if (p.get_name() == "geofence") {
      geofence_ = p.get_value<bool>();
      RCLCPP_INFO(this->get_logger(), "init geofence = %d", geofence_);
    }
  }
  
  mission_control_PEH_ptr_ =
      std::make_shared<rclcpp::ParameterEventHandler>(this);
  
  auto cb_offboard_enable = [this](const rclcpp::Parameter& p) {
    offboard_enable_ = p.get_value<bool>();
    RCLCPP_INFO(this->get_logger(), "received offboard_enable = %d", offboard_enable_);
  };
  auto cb_takeoff = [this](const rclcpp::Parameter& p) {
    takeoff_ = p.get_value<bool>();
    RCLCPP_INFO(this->get_logger(), "received takeoff = %d", takeoff_);
  };
  auto cb_land = [this](const rclcpp::Parameter& p) {
    land_ = p.get_value<bool>();
    RCLCPP_INFO(this->get_logger(), "received land = %d", land_);
  };
  auto cb_geofence = [this](const rclcpp::Parameter& p) {
    geofence_ = p.get_value<bool>();
    RCLCPP_INFO(this->get_logger(), "received geofence = %d", geofence_);
  };
  handle_offboard_enable_= mission_control_PEH_ptr_->add_parameter_callback(
      "offboard_enable", cb_offboard_enable, "/mission_control");
  handle_takeoff_= mission_control_PEH_ptr_->add_parameter_callback(
      "takeoff", cb_takeoff, "/mission_control");
  handle_land_= mission_control_PEH_ptr_->add_parameter_callback(
      "land", cb_land, "/mission_control");
  handle_geofence_ = mission_control_PEH_ptr_->add_parameter_callback(
      "geofence", cb_geofence, "/mission_control");
}

void StarlingOffboard::GetMissionOriginGPS() {
  std::string service_name = "/mission_origin_gps/get_parameters";
  auto mission_origin_parameters_client =
      this->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  while (!mission_origin_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
  auto request =
      std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"mission_origin_lat", "mission_origin_lon", "heading"};
  while (!origin_gps_received_) {
    auto result_future = mission_origin_parameters_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      rclcpp::sleep_for(1s);
      continue;
    }
    auto result = result_future.get();
    if (result->values.size() == 0) {
      rclcpp::sleep_for(1s);
      continue;
    }
    mission_origin_lat_ = result->values[0].double_value;
    mission_origin_lon_ = result->values[1].double_value;
    heading_ = result->values[2].double_value;
    origin_gps_received_ = true;
  }
}


void StarlingOffboard::GetSystemInfo() {
   //   std::string service_name = "/sim/get_system_info";
   //   auto client = this->create_client<async_pac_gnn_interfaces::srv::SystemInfo>(service_name);
   //   while (!client->wait_for_service(1s)) {
   //       if (!rclcpp::ok()) {
   //           rclcpp::shutdown();
   //       }
   //   }
   //   while(!system_info_received_){
   //       auto request = std::make_shared<async_pac_gnn_interfaces::srv::SystemInfo::Request>();
   //       request->name = "starling_offboard";
   //       auto result_future = client->async_send_request(request);
   //       if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
   //           rclcpp::sleep_for(1s);
   //           continue;
   //       }
   //       auto result = result_future.get();
   //       vel_scale_factor_ = result->velocity_scale_factor;
   //       env_scale_factor_ = result->env_scale_factor;
   //       params_.world_size = static_cast<double>(result->world_size);
   //       system_info_received_ = true;
   //   }
   // RCLCPP_WARN(this->get_logger(), "Received system info");
   //
  using async_pac_gnn_interfaces::srv::SystemInfo;
  using ServiceResponseFuture = rclcpp::Client<SystemInfo>::SharedFutureWithRequest;

  if (sys_req_pending_) {
      return;
  }
  if (system_info_received_) {
      return;
  }
  auto request = std::make_shared<SystemInfo::Request>();
  request->name = "starling_offboard";
  RCLCPP_INFO(this->get_logger(), "Request name set to: '%s'", request->name.c_str());

  if (!sys_info_client_){
      RCLCPP_ERROR(this->get_logger(), "sys info client is null");
  }

  sys_req_pending_ = true;
  auto future = sys_info_client_->async_send_request(
    request,
    [this](ServiceResponseFuture future) {
      RCLCPP_ERROR(this->get_logger(), "Sys info callback");
      try {
        sys_req_pending_ = false;
        auto result = future.get();
        auto response = result.second;
        vel_scale_factor_ = response->velocity_scale_factor;
        env_scale_factor_ = response->env_scale_factor;
        params_.world_size = static_cast<double>(response->world_size);
        system_info_received_ = true;
      } 
      catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception thrown: %s", e.what());
      }
    }
  );
  RCLCPP_INFO(this->get_logger(), "Request sent, future valid: %s", future.valid() ? "true" : "false");
}

void StarlingOffboard::GetLaunchGPS() {
  //std::string service_name = "gps_fix/get_parameters";
  //auto homify_parameters_client =
  //    this->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  //while (!homify_parameters_client->wait_for_service(1s)) {
  //  if (!rclcpp::ok()) {
  //    rclcpp::shutdown();
  //  }
  //}
  //while (!gps_received_) {
  //  auto request =
  //      std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  //  request->names = {"launch_gps"};
  //  auto result_future = homify_parameters_client->async_send_request(request);
  //  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
  //                                         result_future) !=
  //      rclcpp::FutureReturnCode::SUCCESS) {
  //    rclcpp::sleep_for(1s);
  //    continue;
  //  }
  //  auto result = result_future.get();
  //  if (result->values.size() == 0) {
  //    rclcpp::sleep_for(1s);
  //    continue;
  //  }
  //  std::vector<double> launch_gps = result->values[0].double_array_value;
  //  launch_gps_lat_ = launch_gps[0];
  //  launch_gps_lon_ = launch_gps[1];
  //  yaw_ = launch_gps[2];
  //  // Flip z-axis to match mission frame
  //  alt_offset_ =  std::max(-launch_gps[3], 0.0);
  //  gps_received_ = true;
  //}
  //
  //
  if (gps_req_pending_) {
      return;
  }

  if (gps_received_) {
    return;
  }
  
  gps_req_pending_ = true;
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"launch_gps"};

  homify_parameters_client_->async_send_request(
    request,
    [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
      gps_req_pending_ = false;
      try {
        auto result = future.get();         
        if (result->values.size() == 0) {
            return;
        }
        std::vector<double> launch_gps = result->values[0].double_array_value;
        launch_gps_lat_ = launch_gps[0];
        launch_gps_lon_ = launch_gps[1];
        yaw_ = launch_gps[2];
        // Flip z-axis to match mission frame
        alt_offset_ =  std::max(-launch_gps[3], 0.0);
        gps_received_ = true;
      }
      catch(const std::exception &e){
        RCLCPP_ERROR(this->get_logger(), "Exception thrown: %s", e.what());
      }
    }
  );
}

void StarlingOffboard::ComputeTransforms() {
  // Compute the translation from the home position to the current (start
  // up position)
  double distance;
  double azimuth_origin_to_target;
  double azimuth_target_to_origin;

  const GeographicLib::Geodesic geod = GeographicLib::Geodesic::WGS84();
  geod.Inverse(mission_origin_lat_, mission_origin_lon_, launch_gps_lat_,
               launch_gps_lon_, distance, azimuth_origin_to_target,
               azimuth_target_to_origin);

  // RCLCPP_INFO(this->get_logger(), "Distance to origin: %f", distance);
  // RCLCPP_INFO(this->get_logger(), "Azimuth origin to target: %f",
  //             azimuth_origin_to_target);
  // RCLCPP_INFO(this->get_logger(), "Azimuth target to origin: %f",
  //             azimuth_target_to_origin);

  double x = distance * cos(azimuth_origin_to_target * M_PI / 180.0);
  double y = distance * sin(azimuth_origin_to_target * M_PI / 180.0);
  double z = 0.0;

  Eigen::Matrix3d rot_mat_z =
      Eigen::AngleAxisd(3. * M_PI / 2. - heading_, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Matrix3d rot_mat_x =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

  Eigen::Matrix3d rot_mat = rot_mat_x * rot_mat_z;

  T_ned_miss_.block<3, 3>(0, 0) = rot_mat;
  T_miss_ned_.block<3, 3>(0, 0) = T_ned_miss_.block<3, 3>(0, 0).transpose();

  Eigen::Vector3d translation = Eigen::Vector3d(x, y, z);
  T_miss_ned_.block<3, 1>(0, 3) = -translation;

  T_ned_miss_ = T_miss_ned_.inverse();


  // Testing of the transformations
  assert(
      (T_miss_ned_ * T_ned_miss_).isApprox(Eigen::Matrix4d::Identity(), 0.001));

  RCLCPP_DEBUG(this->get_logger(), "T_miss_ned:\n%s",
              EigenMatToStr(T_miss_ned_).c_str());
  RCLCPP_DEBUG(this->get_logger(), "T_ned_miss:\n%s",
              EigenMatToStr(T_ned_miss_).c_str());

  RCLCPP_DEBUG(this->get_logger(), "Translation: %f, %f, %f", x, y, z);
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
  subs_.mission_origin_gps = 
      this->create_subscription<geometry_msgs::msg::Point>(
              "/mission_origin_gps", qos_,
              [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                mission_origin_lon_ = msg->x;
                mission_origin_lat_ = msg->y;
                heading_ = msg->z;
                origin_gps_received_ = true;
              });
  subs_.mission_control = 
      this->create_subscription<async_pac_gnn_interfaces::msg::MissionControl>(
              "/mission_control", 10,
              [this](const async_pac_gnn_interfaces::msg::MissionControl::SharedPtr msg) {
                offboard_enable_ = msg->offboard_enable;
                takeoff_ = msg->takeoff;
                land_ = msg->land;
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
      1000ms, std::bind(&StarlingOffboard::StatusTimerCallback, this));
}

/**
 * @brief Main Loop
 */
void StarlingOffboard::TimerCallback() {

  // Geofence check
  if (init_done_) {
    GeofenceCheck();
  }

  // State Machine
  switch (state_) {
    case State::INIT_START: {
      // Services
      InitializeSubscribers();
      InitializePublishers();
      RCLCPP_WARN(this->get_logger(), "Initialized pubs and subs");
      state_ = State::INIT_GPS;
      break;
    }

    case State::INIT_GPS: {
      if (homify_parameters_client_->wait_for_service(1s)) {
        GetLaunchGPS();
        if (gps_received_) {
          RCLCPP_WARN(this->get_logger(), "Launch GPS received");
          state_ = State::INIT_SYS;
        }
      }
      else{;
          RCLCPP_ERROR_ONCE(this->get_logger(), "Parameter service not available. Trying again");
      }
      break;
    }

    case State::INIT_SYS: {
      if (sys_info_client_->wait_for_service(1s)) {
        GetSystemInfo();
        if (system_info_received_) {
          RCLCPP_WARN(this->get_logger(), "System info received from GCS");
          RCLCPP_WARN(this->get_logger(), "Environment scale factor: %f", env_scale_factor_);
          state_ = State::INIT_FIN;
        }
      }
      else{
          RCLCPP_ERROR(this->get_logger(), "System info service not available. Trying again.");
      }
      break;
    }

    case State::INIT_FIN: {
      InitializeGeofence();
      init_done_ = true;
      state_ = State::IDLE;
      break;
    }

    case State::IDLE: {
      // Mission origin has been converted to a topic
      // Homify launch GPS is still a parameter
      // Can't get to this point without having received it
      if (!mission_control_received_) {
          RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for mission control topic...");
          break;
      }
      RCLCPP_INFO_ONCE(this->get_logger(), "Mission control received.");

      if (!origin_gps_received_) {
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

      ComputeTransforms();

      Eigen::Vector4d current_mission_pos =
          TransformVec(Eigen::Vector4d(pos_msg_.x, pos_msg_.y, pos_msg_.z, 1.0),
                       T_ned_miss_);
      RCLCPP_INFO(this->get_logger(), "Current mission pos: %f, %f, %f",
                  current_mission_pos[0], current_mission_pos[1],
                  current_mission_pos[2]);

      takeoff_pos_[0] = current_mission_pos[0];
      takeoff_pos_[1] = current_mission_pos[1];

      takeoff_pos_ned_ = TransformVec(takeoff_pos_, T_miss_ned_);

      RCLCPP_DEBUG(this->get_logger(), "Takeoff pos (miss): %f, %f, %f",
                  takeoff_pos_[0], takeoff_pos_[1], takeoff_pos_[2]);
      RCLCPP_DEBUG(this->get_logger(), "Takeoff pos (ned): %f, %f, %f",
                  takeoff_pos_ned_[0], takeoff_pos_ned_[1],
                  takeoff_pos_ned_[2]);

      Eigen::Vector4d takeoff_pos_check =
          TransformVec(takeoff_pos_ned_, T_ned_miss_);
      RCLCPP_DEBUG(this->get_logger(), "Takeoff pos check: %f, %f, %f",
                  takeoff_pos_check[0], takeoff_pos_check[1],
                  takeoff_pos_check[2]);

      assert(TransformVec(takeoff_pos_ned_, T_ned_miss_)
                 .isApprox(takeoff_pos_, 0.1));

      start_pos_ned_[0] = takeoff_pos_ned_[0];
      start_pos_ned_[1] = takeoff_pos_ned_[1];

      RCLCPP_WARN_ONCE(this->get_logger(), "kP: %f", params_.kP);

      state_ = State::PREFLT;
      RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
      break;
    }

    case State::PREFLT:
      if (takeoff_ && offboard_enable_) {

        state_ = State::ARMING;
        RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
        RCLCPP_DEBUG(this->get_logger(), "yaw_: %f", yaw_);
      } 
      break;

    case State::ARMING:
      if (offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        this->PubVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
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

      // stop the counter after reaching 11
      if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
      }
      break;
    

    case State::TAKEOFF:
      // error calculation
      takeoff_completed_ = this->HasReachedPos(takeoff_pos_ned_);

      if (takeoff_completed_) {
        PubOffboardControlMode(ControlMode::VEL);
        PubTrajSetpointVel(stop_vel_);
        state_ = State::MISSION;
        RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Takeoff complete -- reached setpoint within TOL");
      } else {
        // offboard_control_mode needs to be paired with trajectory_setpoint
        PubOffboardControlMode(ControlMode::POS);
        PubTrajSetpointPos(takeoff_pos_ned_);
      }
      break;
    
    // GNN, Square, etc..
    case State::MISSION: {
      if (land_) {
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
        ClampVelocity(stop_vel_);
        PubTrajSetpointVel(stop_vel_);
        //RCLCPP_INFO(this->get_logger(), "Mission velocity update timeout; stop velocity (%f, %f, %f)", stop_vel_[0], stop_vel_[1], stop_vel_[2]);
      }

      else {
        // RCLCPP_INFO(this->get_logger(), "NED Velocity (%f, %f, %f)",
        // vel_ned_[0], vel_ned_[1], vel_ned_[2]);
        PubTrajSetpointVel(vel_ned_);
      }
      break;
    }

    case State::LANDING_H:
      reached_land_pos_h_ = this->HasReachedPos(takeoff_pos_ned_);
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
      // Check both position and velocity to ensure the drone has landed
      reached_land_pos_v_ = this->HasReachedPos(start_pos_ned_);
      reached_land_stationary_v_ = std::abs(pos_msg_.vz) < 0.1;

      if (reached_land_pos_v_ && reached_land_stationary_v_) {
        state_ =  State::DISARM;
        RCLCPP_WARN_ONCE(this->get_logger(), "Landing sequence finished");
        RCLCPP_INFO(this->get_logger(), "State: %s", StateToString(state_).c_str());
      }
      else{
        PubOffboardControlMode(ControlMode::VEL);
        PubTrajSetpointPos(land_vel_);
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
 * @brief Compute the velocity to reach the target position
 */
Eigen::Vector4d StarlingOffboard::ComputeVel(
    const Eigen::Vector4d& target_pos) {
  const double kP = 1.0;

  const double err_x = (pos_msg_.x - target_pos[0]);
  const double err_y = (pos_msg_.y - target_pos[1]);
  const double err_z = (pos_msg_.z - target_pos[2]);

  Eigen::Vector4d vel;
  vel << -kP * err_x, -kP * err_y, -kP * err_z, 0.0;
  return vel;
}

/**
 * @brief Check if the drone has reached the target position within tolerance
 */
bool StarlingOffboard::HasReachedPos(const Eigen::Vector4d& target_pos) {
  const double err_x = std::abs(pos_msg_.x - target_pos[0]);
  const double err_y = std::abs(pos_msg_.y - target_pos[1]);
  const double err_z = std::abs(pos_msg_.z - target_pos[2]);

  return err_x < params_.position_tolerance &&
         err_y < params_.position_tolerance &&
         err_z < params_.position_tolerance;
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
  if (offboard_enable_) {
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
  if (offboard_enable_ && !breach_) {
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
  if (offboard_enable_) {
    pubs_.traj_setpoint->publish(msg);
  }
}

/**
 * @brief Publish the trajectory setpoint (TwistStamped) to the PX4
 */
void StarlingOffboard::UpdateVel(
  const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel) {
  // Proportional controller to maintain altitude
  const double kP = 1.0;
  const double err_z = (params_.z_takeoff + pos_msg_.z);

  // const Eigen::Vector4d vel_mission (
  //                           (double) clamp(scale *
  //                           cmd_vel->twist.linear.y, -2.0, 2.0), (double)
  //                           clamp(scale * cmd_vel->twist.linear.x,
  //                           -2.0, 2.0), (double) clamp((double)(-kP * err_z),
  //                           -2.0, 2.0), 1.0);
  const Eigen::Vector4d vel_mission(cmd_vel->twist.linear.x,
                                    cmd_vel->twist.linear.y, 
                                    kP * err_z, 
                                    0.0);

  // Transform the velocity from the mission frame to NED
  vel_ned_ = TransformVec(vel_mission, T_miss_ned_);
  ClampVelocity(vel_ned_);
  time_last_vel_update_ = clock_->now();
}

/**
 * @brief Publish the pose (PoseStamped) to the GNN. Publishing the path as well
 * for visualization
 */
void StarlingOffboard::VehicleLocalPosCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr pos_msg) {
  // Only publish the pose once the drone is armed and transforms have been set
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

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StarlingOffboard>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
