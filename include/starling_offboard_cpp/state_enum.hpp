#pragma once

#include <rclcpp/rclcpp.hpp>
namespace pac_ws::starling_offboard {

enum class ServiceState { START, REQ_SENT, FINAL };

template <typename T>
struct ServiceClient {
 public:
  std::string service_name;
  ServiceState state = ServiceState::START;
  typename rclcpp::Client<T>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;

  ServiceClient() = default;
  explicit ServiceClient(std::string const &name) : service_name{name} {}
  ServiceClient(const ServiceClient&)            = delete;
  ServiceClient& operator=(const ServiceClient&) = delete;
  ServiceClient(ServiceClient&&)                 = default;
  ServiceClient& operator=(ServiceClient&&)      = default;

  bool Done() {
    return state == ServiceState::FINAL;
  }

};

enum class State {
  INIT_START,
  INIT_GPS,
  IDLE,
  PREFLT,
  ARMING,
  TAKEOFF,
  MISSION,
  LANDING_H,
  LANDING_V,
  DISARM
};

inline std::string StateToString(State state) {
  switch (state) {
    case State::INIT_START:
      return "INIT_START";
    case State::INIT_GPS:
      return "INIT_GPS";
    case State::IDLE:
      return "IDLE";
    case State::PREFLT:
      return "PREFLT";
    case State::ARMING:
      return "ARMING";
    case State::TAKEOFF:
      return "TAKEOFF";
    case State::MISSION:
      return "MISSION";
    case State::LANDING_H:
      return "LANDING_H";
    case State::LANDING_V:
      return "LANDING_V";
    case State::DISARM:
      return "DISARM";
    default:
      return "INVALID";
  }
}

inline std::ostream& operator<<(std::ostream& os, State state) {
  return os << StateToString(state);
}
}  // namespace pac_ws::starling_offboard
