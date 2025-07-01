#pragma once

#include <Eigen/Dense>
#include <px4_msgs/msg/vehicle_local_position.hpp>

namespace pac_ws::starling_offboard {
/**
 * @brief Compute the velocity to reach the target position
 */
inline Eigen::Vector4d PDController(
    px4_msgs::msg::VehicleLocalPosition curr_pos,
    const Eigen::Vector4d& target_pos, const double kP = 1.0) {
  double err_x = (curr_pos.x - target_pos[0]);
  double err_y = (curr_pos.y - target_pos[1]);
  double err_z = (curr_pos.z - target_pos[2]);

  Eigen::Vector4d vel;
  vel << -kP * err_x, -kP * err_y, -kP * err_z, 0.0;
  return vel;
}

/**
 * @brief Check if the drone has reached the target position within tolerance
 */
inline bool HasReachedPos(px4_msgs::msg::VehicleLocalPosition curr_pos,
                          const Eigen::Vector4d& target_pos,
                          double tolerance = 1.0) {
  const double err_x = std::abs(curr_pos.x - target_pos[0]);
  const double err_y = std::abs(curr_pos.y - target_pos[1]);
  const double err_z = std::abs(curr_pos.z - target_pos[2]);

  return err_x < tolerance && err_y < tolerance && err_z < tolerance;
}

inline void ClampVelocity(const double max_speed,
                          Eigen::Vector4d& vel) noexcept {
  vel.x() = std::clamp(vel.x(), -max_speed, max_speed);
  vel.y() = std::clamp(vel.y(), -max_speed, max_speed);
  vel.z() = std::clamp(vel.z(), -max_speed, max_speed);
}

inline std::string EigenToStr(const Eigen::Matrix<double, 4, 4>& mat) {
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "  [",
                           "]", "[\n", "\n]");

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4) << mat.format(CleanFmt);
  return oss.str();
}

inline std::string EigenToStr(const Eigen::Vector4d& vec) {
  Eigen::IOFormat CleanFmt(4, 0, ", ", ", ", "[",
                           "]");

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4) << vec.format(CleanFmt);

  return oss.str();
}

inline Eigen::Vector4d TransformVec(const Eigen::Vector4d& vec,
                                    const Eigen::Matrix<double, 4, 4>& Tf) {
  Eigen::Vector4d transformed_vec = Tf * vec;
  return transformed_vec;
}
}  // namespace pac_ws::starling_offboard
