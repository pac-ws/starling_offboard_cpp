#include "starling_offboard_cpp/starling_offboard.hpp"

namespace pac_ws::starling_offboard {

void StarlingOffboard::GetLaunchGPS() {
  using ServiceType = rcl_interfaces::srv::GetParameters;
  auto &sc = launch_gps_sc_;

  if(!sc) {
    std::string launch_service_name = "gps_fix/get_parameters";
    sc = std::make_shared<ServiceClient<ServiceType>>(launch_service_name);
    sc->client = this->create_client<ServiceType>(sc->service_name);
    sc->state = ServiceState::START;
    sc->timer = this->create_wall_timer(
        intervals_.Long, std::bind(&StarlingOffboard::GetLaunchGPS, this));
  }

  if (sc->state == ServiceState::START) {
    if (sc->client->wait_for_service(intervals_.Short)) {
      auto cb = [
        &lat = launch_gps_lat_,
        &lon = launch_gps_lon_,
        &yaw = yaw_,
        &alt = alt_offset_, sc](rclcpp::Client<ServiceType>::SharedFuture future) {
        auto result = future.get();
        if (result->values.size() == 0) {
          sc->state = ServiceState::START;
          return;
        }
        std::vector<double> launch_gps = result->values[0].double_array_value;
        lat = launch_gps[0];
        lon = launch_gps[1];
        yaw = launch_gps[2];
        // Flip z-axis to match mission frame
        alt = std::max(-launch_gps[3], 0.0);
        sc->timer->cancel();
        sc->timer.reset();
        sc->client.reset();
        sc->state = ServiceState::FINAL;
      };

      auto request = std::make_shared<ServiceType::Request>();
      request->names = {"launch_gps"};
      sc->state = ServiceState::REQ_SENT;
      sc->client->async_send_request(request, std::move(cb));
    }
  }
}

void StarlingOffboard::GetSystemInfo() {
  using ServiceType = async_pac_gnn_interfaces::srv::SystemInfo;
  auto &sc = system_info_sc_;

  if(!sc) {
    std::string sys_info_service_name = "/sim/get_system_info";
    sc = std::make_shared<ServiceClient<ServiceType>>(sys_info_service_name);
    sc->client = this->create_client<ServiceType>(sc->service_name);
    sc->state = ServiceState::START;
    sc->timer = this->create_wall_timer(
        intervals_.Long, std::bind(&StarlingOffboard::GetSystemInfo, this));
  }

  if (sc->state == ServiceState::START) {
    if (sc->client->wait_for_service(intervals_.Short)) {
      auto cb = [
        &v_sc = vel_scale_factor_,
        &e_sc = env_scale_factor_,
        &w_sz = params_.world_size,
        sc](rclcpp::Client<ServiceType>::SharedFuture future) {
        auto result = future.get();
        v_sc = result->velocity_scale_factor;
        e_sc = result->env_scale_factor;
        w_sz = static_cast<double>(result->world_size);
        sc->timer->cancel();
        sc->timer.reset();
        sc->client.reset();
        sc->state = ServiceState::FINAL;
      };

      auto request = std::make_shared<ServiceType::Request>();
      request->name = "starling_offboard";
      sc->state = ServiceState::REQ_SENT;
      sc->client->async_send_request(request, std::move(cb));
    }
  }
}

void StarlingOffboard::GetMissionOriginGPS() {
  using ServiceType = rcl_interfaces::srv::GetParameters;
  auto &sc = mission_origin_gps_sc_;

  if(!sc) {
    std::string service_name = "/mission_origin_gps/get_parameters";
    sc = std::make_shared<ServiceClient<ServiceType>>(service_name);
    sc->client = this->create_client<ServiceType>(sc->service_name);
    sc->state = ServiceState::START;
    sc->timer = this->create_wall_timer(
        intervals_.Long, std::bind(&StarlingOffboard::GetMissionOriginGPS, this));
  }

  if (sc->state == ServiceState::START) {
    if (sc->client->wait_for_service(intervals_.Short)) {
      auto cb = [
        &lat = mission_origin_lat_,
        &lon = mission_origin_lon_,
        &heading = heading_,
        sc](rclcpp::Client<ServiceType>::SharedFuture future) {
        auto result = future.get();
        if (result->values.size() == 0) {
          sc->state = ServiceState::START;
          return;
        }
        std::vector<double> launch_gps = result->values[0].double_array_value;
        lat = result->values[0].double_value;
        lon = result->values[1].double_value;
        heading = result->values[2].double_value;
        sc->timer->cancel();
        sc->timer.reset();
        sc->client.reset();
        sc->state = ServiceState::FINAL;
      };

      auto request = std::make_shared<ServiceType::Request>();
      request->names = {"mission_origin_lat", "mission_origin_lon", "heading"};
      sc->state = ServiceState::REQ_SENT;
      sc->client->async_send_request(request, std::move(cb));
    }
  }
}
}  // namespace pac_ws::starling_offboard
