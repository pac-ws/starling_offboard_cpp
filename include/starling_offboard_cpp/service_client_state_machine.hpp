#pragma once

namespace pac_ws::starling_offboard {
template <typename T>
class ServiceClientStateMachine {
 public:
  enum class ServiceState { START, INIT_DONE, REQ_SENT, FINAL };

  rclcpp::Client<T>::SharedPtr client;
  ServiceState state = ServiceState::START;
};
}  // namespace pac_ws::starling_offboard
