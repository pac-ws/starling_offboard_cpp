
void StarlingOffboard::GetMissionOriginGPS() {
  std::string service_name = "/mission_origin_gps/get_parameters";
  auto mission_origin_parameters_client =
      this->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  while (!mission_origin_parameters_client->wait_for_service(intervals_.Mid)) {
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
      rclcpp::sleep_for(intervals_.Mid);
      continue;
    }
    auto result = result_future.get();
    if (result->values.size() == 0) {
      rclcpp::sleep_for(intervals_.Mid);
      continue;
    }
    mission_origin_lat_ = result->values[0].double_value;
    mission_origin_lon_ = result->values[1].double_value;
    heading_ = result->values[2].double_value;
    origin_gps_received_ = true;
  }
}

void StarlingOffboard::GetMissionControl(){
  sync_parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/mission_control");
  while (!sync_parameters_client_->wait_for_service(intervals_.Mid)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  
  auto result = sync_parameters_client_->get_parameters({"ob_enable", "takeoff", "land", "geofence"});
  RCLCPP_DEBUG(this->get_logger(), "result size = %ld", result.size());
  for (auto &p : result) {
    if (p.get_name() == "ob_enable") {
      ob_enable_ = p.get_value<bool>();
      RCLCPP_INFO(this->get_logger(), "init ob_enable = %d", ob_enable_);
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
  
  auto cb_ob_enable = [this](const rclcpp::Parameter& p) {
    ob_enable_ = p.get_value<bool>();
    RCLCPP_INFO(this->get_logger(), "received ob_enable = %d", ob_enable_);
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
  handle_ob_enable_= mission_control_PEH_ptr_->add_parameter_callback(
      "ob_enable", cb_ob_enable, "/mission_control");
  handle_takeoff_= mission_control_PEH_ptr_->add_parameter_callback(
      "takeoff", cb_takeoff, "/mission_control");
  handle_land_= mission_control_PEH_ptr_->add_parameter_callback(
      "land", cb_land, "/mission_control");
  handle_geofence_ = mission_control_PEH_ptr_->add_parameter_callback(
      "geofence", cb_geofence, "/mission_control");
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
}
