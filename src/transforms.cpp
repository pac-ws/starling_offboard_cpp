#include <GeographicLib/Geodesic.hpp>

#include "starling_offboard_cpp/starling_offboard.hpp"

namespace pac_ws::starling_offboard {
void StarlingOffboard::ComputeLocalMissionTransform() {
  double distance;
  double azimuth_origin_to_target;
  double azimuth_target_to_origin;

  const GeographicLib::Geodesic geod = GeographicLib::Geodesic::WGS84();
  geod.Inverse(mission_origin_lat_,
               mission_origin_lon_,
               launch_gps_lat_,
               launch_gps_lon_,
               distance,
               azimuth_origin_to_target,
               azimuth_target_to_origin);

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

  assert((T_miss_ned_ * T_ned_miss_).isApprox(Eigen::Matrix4d::Identity(), 0.001));
  RCLCPP_DEBUG(this->get_logger(), "T_miss_ned:\n%s", EigenToStr(T_miss_ned_).c_str());
  RCLCPP_DEBUG(this->get_logger(), "T_ned_miss:\n%s", EigenToStr(T_ned_miss_).c_str());
  RCLCPP_DEBUG(this->get_logger(), "Translation: %f, %f, %f", x, y, z);
}

void StarlingOffboard::ComputeExtrinsicTransforms() {
  T_imu_cam_ << -1, 0, 0, -0.08825, 0, -1, 0, -0.0045, 0, 0, 1, 0.00269, 0, 0, 0, 1;
  T_cam_imu_ = T_imu_cam_.inverse();
  T_body_imu_ << 1, 0, 0, 0.0295, 0, 1, 0, -0.0065, 0, 0, 1, -0.016, 0, 0, 0, 1;
  T_imu_body_ = T_body_imu_.inverse();
}

void StarlingOffboard::ComputeLocalBodyTransform() {
  Eigen::Quaterniond q(att_msg_.q[0], att_msg_.q[1], att_msg_.q[2], att_msg_.q[3]); // wxyz
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d t(pos_msg_.x, pos_msg_.y, pos_msg_.z);
  T_body_imu_.block<3, 3>(0, 0) = R;
  T_body_imu_.block<3, 1>(0, 3) = t;
  T_imu_body_ = T_body_imu_.inverse();
}

void StarlingOffboard::ComputeTagCamTransform() {
  if (!tf_tag_cam_received_) {
    return;
  }
  geometry_msgs::msg::TransformStamped tag_cam = tf_tag_cam_msg_.transforms[0];
  Eigen::Isometry3d iso = tf2::transformToEigen(tag_cam);
  T_tag_cam_ = iso.matrix();
  T_cam_tag_ = T_tag_cam_.inverse();
}

void StarlingOffboard::ComputeTagLocalTransform() {
  T_tag_ned_ = T_body_ned_ * T_imu_body_ * T_cam_imu_ * T_tag_cam_;
  T_ned_tag_ = T_tag_ned_.inverse();
}

void StarlingOffboard::ComputeTagBodyTransform() {
  T_tag_body_ = T_imu_body_ * T_cam_imu_ * T_tag_cam_;
  T_body_tag_ = T_tag_body_.inverse();
}

// DEPRECATED
//void StarlingOffboard::ComputeTagNedTransform() {
//  // The AprilTag is located underneath the drone at takeoff.
//  // Assume that the front of the drone xB is aligned with -yT the top of the
//  // AprilTag Then T_tag_ned is defined by a rotation about the z-axis given by
//  // the heading.
//  Eigen::Matrix3d rot_mat_z =
//      Eigen::AngleAxisd(-M_PI / 2 - azimuth_, Eigen::Vector3d::UnitZ())
//          .toRotationMatrix();
//  T_tag_ned_.block<3, 3>(0, 0) = rot_mat_z;
//  T_ned_tag_ = T_tag_ned_.inverse();
//  RCLCPP_WARN_ONCE(this->get_logger(), "Azimuth: %f", azimuth_);
//  RCLCPP_WARN_ONCE(this->get_logger(), "T_tag_ned_:\n%s",
//                   EigenToStr(T_tag_ned_).c_str());
//}

void StarlingOffboard::ComputeStartPosTakeoff() {
  Eigen::Vector4d current_mission_pos = TransformVec(
      Eigen::Vector4d(pos_msg_.x, pos_msg_.y, pos_msg_.z, 1.0), T_ned_miss_);
  RCLCPP_INFO(this->get_logger(), "Current mission pos: %s",
              EigenToStr(current_mission_pos).c_str());

  takeoff_pos_[0] = current_mission_pos[0];
  takeoff_pos_[1] = current_mission_pos[1];
  takeoff_pos_[2] = params_.z_takeoff + alt_offset_;
  takeoff_pos_[3] = 1.0;

  RCLCPP_DEBUG(this->get_logger(), "Takeoff position: %s",
               EigenToStr(takeoff_pos_).c_str());

  takeoff_pos_ned_ = TransformVec(takeoff_pos_, T_miss_ned_);

  RCLCPP_DEBUG(this->get_logger(), "Takeoff pos (ned): %s",
               EigenToStr(takeoff_pos_ned_).c_str());

  Eigen::Vector4d takeoff_pos_check =
      TransformVec(takeoff_pos_ned_, T_ned_miss_);
  RCLCPP_DEBUG(this->get_logger(), "Takeoff pos check: %s",
               EigenToStr(takeoff_pos_check).c_str());

  assert(
      TransformVec(takeoff_pos_ned_, T_ned_miss_).isApprox(takeoff_pos_, 0.1));

  start_pos_ned_[0] = takeoff_pos_ned_[0];
  start_pos_ned_[1] = takeoff_pos_ned_[1];
}
} // namespace pac_ws::starling_offboard
