#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Origins and offsets
#define LAT_HOME 39.941349055103075
#define LON_HOME -75.19878530679716
#define ALT_HOME 11.745387077331543
#define HEADING_NED_TO_FRD 2.725

#define _USE_MATH_DEFINES
#include <math.h>

// Square mission parameters
#define S_Z -1.0
#define S_X 2.0

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum class State {
	IDLE,
    ARMING,
	TAKEOFF,
	MISSION,
	LANDING
};

const std::string to_string(State state){
    switch(state){
        case State::IDLE: return "IDLE";
        case State::ARMING: return "ARMING";
        case State::TAKEOFF: return "TAKEOFF";
        case State::MISSION: return "MISSION";
        case State::LANDING: return "LANDING";
        default: return "INVALID";
    }
}

const unsigned char* toChar(State state){
	switch(state){
		case State::IDLE: return reinterpret_cast<const unsigned char*>("IDLE");
		case State::ARMING: return reinterpret_cast<const unsigned char*>("ARMING");
		case State::TAKEOFF: return reinterpret_cast<const unsigned char*>("TAKEOFF");
		case State::MISSION: return reinterpret_cast<const unsigned char*>("MISSION");
		case State::LANDING: return reinterpret_cast<const unsigned char*>("LANDING");
		default: return reinterpret_cast<const unsigned char*>("INVALID");
	}
}

template <typename T>
T clamp(T val, T min, T max){
    return std::max(min, std::min(val, max));
}

std::ostream& operator<<(std::ostream& os, const unsigned char* state){
	return os << reinterpret_cast<const char*>(state);
}

std::ostream& operator<<(std::ostream& os, State state){
	return os << toChar(state);
}

class StarlingOffboard: public rclcpp::Node
{
public:
	StarlingOffboard() : Node("starling_offboard")
	{
        // Parameters
        this->declare_parameter<float>("takeoff_z", -1.0);
        this->get_parameter("takeoff_z", takeoff_z);

        this->declare_parameter<float>("scale", 1.0);
        this->get_parameter("scale", scale);

        // For consistency...
        clock_ = this->get_clock();

        // Number of waypoints to set before attempting to enter offboard mode
		offboard_setpoint_counter_ = 0;
        
        // Transformation matrix from Mission to NED
        inv_rotation = Eigen::AngleAxisf(-HEADING_NED_TO_FRD, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        T_miss_ned.block<3,3>(0,0) = inv_rotation;

        // Transformation matrix from NED to Mission
        rotation = Eigen::AngleAxisf(HEADING_NED_TO_FRD, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        T_ned_miss.block<3,3>(0,0) = rotation;

        takeoff_pos << 0.0, 0.0, takeoff_z, 0.0;

        // Used to stop the drone when it reaches the waypoint
        stop_vel << 0.0, 0.0, 0.0, 1.0;

        // Holds the current velocity from the mission to be sent to the px4
        vel_ned << 0.0, 0.0, 0.0, 1.0;

        // QoS
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        // Pubs and Subs
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", qos);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("fmu/in/vehicle_command", qos);
        drone_status_publisher_ = this->create_publisher<std_msgs::msg::String>("drone_status", qos);

        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("fmu/out/vehicle_status", qos, [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg){
            arming_state_ = msg->arming_state;
        });

		pos_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("fmu/out/vehicle_local_position", qos, [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg){
			pos_msg_ = *msg;
		});

        global_pos_subscription_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>("fmu/out/vehicle_global_position", qos, [this](const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg){
            global_pos_msg_ = *msg;
        });

        takeoff_subscription_ = this->create_subscription<std_msgs::msg::Bool>("takeoff", qos, [this](const std_msgs::msg::Bool::UniquePtr msg){
            takeoff_cmd_received = msg->data;
        });

        // Velocity Translation (TwistStamped [GNN] to TrajectorySetpoint [PX4])
        gnn_vel_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", qos, std::bind(&StarlingOffboard::update_vel, this, std::placeholders::_1));
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/in/trajectory_setpoint", qos);

        // Position Translation (VehicleLocalPosition [PX4] to PoseStamped [GNN])
        vehicle_local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("fmu/out/vehicle_local_position", qos, std::bind(&StarlingOffboard::publish_pose, this, std::placeholders::_1));
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", qos);

        // 10Hz Timer
        timer_ = this->create_wall_timer(100ms, std::bind(&StarlingOffboard::timer_callback, this));
	}

	void arm();
	void disarm();

private:
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
    rclcpp::Time time_last_vel_update;
                                        
    // Timer drives the main loop
	rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;

    // Publishers and Subscribers
    // From the interface
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gnn_vel_subscription_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drone_status_publisher_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_pos_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_subscription_;
	px4_msgs::msg::VehicleLocalPosition pos_msg_;
    px4_msgs::msg::VehicleGlobalPosition global_pos_msg_;
    uint8_t arming_state_;
    bool takeoff_cmd_received = false;

    float takeoff_z;
	bool takeoff = false;
	bool waypt_reached = false;

	// Waypoints are still used as goals
    float scale;
    Eigen::Vector4f vel_ned;
    Eigen::Vector4f stop_vel;
    Eigen::Vector4f takeoff_pos;
    Eigen::Matrix<float, 4, 4> waypts;
	uint8_t way_pt_idx;
	const float POS_TOL_ = 0.5; // waypoint position tolerance in meters
    
    // Transformation matrix from NED to FRD
    double x_offset;
    double y_offset;
    double z_offset;

    Eigen::Vector3f translation;
    Eigen::Vector3f inv_translation;
    Eigen::Matrix3f rotation;
    Eigen::Matrix3f inv_rotation;
    Eigen::Matrix<float, 4, 4> T_miss_ned = Eigen::Matrix4f::Identity();
    Eigen::Matrix<float, 4, 4> T_ned_miss = Eigen::Matrix4f::Identity();

	State state_ = State::IDLE;

    // Function prototypes
    //rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    Eigen::Vector3f compute_translation(const double ref_lat, const double ref_lon, const float ref_alt, const double lat, const double lon, const float alt);
    Eigen::Vector4f compute_vel(const Eigen::Vector4f& target_pos);
    Eigen::Vector4f tform(const Eigen::Vector4f &vec, const Eigen::Matrix<float, 4, 4> &Tf);
	void publish_offboard_control_mode(const bool is_pos, const bool is_vel);
	void publish_trajectory_setpoint_vel(const Eigen::Vector4f& target_vel);
	void publish_trajectory_setpoint_pos(const Eigen::Vector4f& target_pos);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, double param5 = 0.0, double param6 = 0.0, float param7 = 0.0);
	bool has_reached_pos(const Eigen::Vector4f& target_pos);
	void set_home(const double lat, const double lon, const float alt);
    void update_vel(const geometry_msgs::msg::TwistStamped::SharedPtr gnn_cmd_vel);
    void publish_pose(const px4_msgs::msg::VehicleLocalPosition::SharedPtr vehicle_local_position);
    void timer_callback();
};

/**
 * @brief Main Loop
 */
void StarlingOffboard::timer_callback(){

    //std::cout << "State: " << state_ << std::endl;
    // Publish the current state
    auto state_msg = std_msgs::msg::String();
    state_msg.data = to_string(state_);
    drone_status_publisher_->publish(state_msg);

    // State Machine
    switch (state_) {

        case State::IDLE:
            // Compute the translation from the home position to the current (start up position)
            translation = compute_translation(LAT_HOME, LON_HOME, ALT_HOME, global_pos_msg_.lat, global_pos_msg_.lon, global_pos_msg_.alt);
            inv_translation = -translation;

            std::cout << "Translation: " << translation << std::endl;
            
            // Don't need translation for velocity
            //T_miss_ned.block<3,1>(0,3) = inv_translation;
            
            // Need translation for position
            T_ned_miss.block<3,1>(0,3) = translation;

            // TODO 
            /*
            if (takeoff_cmd_received) {
                state_ = State::TAKEOFF;
                takeoff_cmd_received = false;
            }
            */

            // TODO
            state_ = State::ARMING;
            std::cout << "State: " << state_ << std::endl;
            break;

        case State::ARMING:
            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
                
                if (arming_state_ == 2) {
                    RCLCPP_INFO(this->get_logger(), "Vehicle armed");
                    state_ = State::TAKEOFF;
                    std::cout << "State: " << state_ << std::endl;
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "Vehicle not armed");
                    // Retry
                    offboard_setpoint_counter_ = 0;
                }
                    
            }

            // Send 10 setpoints before attempting to arm
            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode(true, false);
            publish_trajectory_setpoint_pos(takeoff_pos);
            
            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
            break;

        case State::LANDING:
        case State::TAKEOFF:
            
            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode(true, false);
            publish_trajectory_setpoint_pos(takeoff_pos);

            // error calculation
            takeoff = this->has_reached_pos(takeoff_pos);
            if (takeoff) {
                publish_trajectory_setpoint_vel(stop_vel);
                state_ = State::MISSION;
                std::cout << "State: " << state_ << std::endl;
                RCLCPP_INFO(this->get_logger(), "Takeoff complete -- reached setpoint within TOL");
            }

            break;

        // GNN, Square, etc..
        case State::MISSION:
            
            // offboard_control_mode needs to be paired with trajectory_setpoint
            // If the message rate drops bellow 2Hz, the drone exits offboard control mode
            publish_offboard_control_mode(false, true);
            
            rclcpp::Duration duration = clock_->now() - time_last_vel_update;
            if (duration.seconds() > 0.5) {
                RCLCPP_INFO(this->get_logger(), "Mission velocity update timeout, sending stop velocity");
                publish_trajectory_setpoint_vel(stop_vel);
            }
            else {
                publish_trajectory_setpoint_vel(vel_ned);
            }
            break;

    }
}

/**
 * @brief Compute the translation from the reference position to the current position
 */
// TODO double check this!!
Eigen::Vector3f StarlingOffboard::compute_translation(const double ref_lat, const double ref_lon, const float ref_alt, const double lat, const double lon, const float alt){
    const double R = 6371000.0; // Earth radius in meters
    const double d_lat = (lat - ref_lat) * M_PI / 180.0;
    const double d_lon = (lon - ref_lon) * M_PI / 180.0;
    const double d_alt = alt - ref_alt;
    const double x = R * d_lat;
    const double y = R * d_lon * cos(lat * M_PI / 180.0);
    const double z = d_alt;
    return Eigen::Vector3f(x, y, z);
}

/**
 * @brief Transform the position from mission frame to NED
 */
Eigen::Vector4f StarlingOffboard::tform(const Eigen::Vector4f &vec_mission, const Eigen::Matrix<float, 4, 4> &Tf){
    Eigen::Vector4f vec_ned = Tf * vec_mission;
    return vec_ned;
}

/**
 * @brief Compute the velocity to reach the target position
 */
Eigen::Vector4f StarlingOffboard::compute_vel(const Eigen::Vector4f& target_pos){
	const float kP = 1.0;

	const float err_x = (pos_msg_.x - target_pos[0]);
	const float err_y = (pos_msg_.y - target_pos[1]);
	const float err_z = (pos_msg_.z - target_pos[2]);

    Eigen::Vector4f vel;
    vel << -kP * err_x, -kP * err_y, -kP * err_z, 0.0;
	return vel;
}

/**
 * @brief Check if the drone has reached the target position within tolerance
 */
bool StarlingOffboard::has_reached_pos(const Eigen::Vector4f& target_pos)
{
	const float err_x = std::abs(pos_msg_.x - target_pos[0]);
	const float err_y = std::abs(pos_msg_.y - target_pos[1]);
	const float err_z = std::abs(pos_msg_.z - target_pos[2]);

	return err_x < POS_TOL_ && err_y < POS_TOL_ && err_z < POS_TOL_;
}
/**
 * @brief Set the home position of the drone
 */
void StarlingOffboard::set_home(const double lat, const double lon, const float alt)
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 0.0, 0.0, lat, lon, alt);
	RCLCPP_INFO(this->get_logger(), "Home position set");
}

/**
 * @brief Send a command to Arm the vehicle
 */
void StarlingOffboard::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, 0.0, 0.0, 0.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void StarlingOffboard::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, 0.0, 0.0, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void StarlingOffboard::publish_offboard_control_mode(const bool is_pos, const bool is_vel)
{
	OffboardControlMode msg{};
	msg.position = is_pos;
	msg.velocity = is_vel;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint (vel)
 */
void StarlingOffboard::publish_trajectory_setpoint_vel(const Eigen::Vector4f& target_vel)
{
	TrajectorySetpoint msg{};
	msg.position = {std::nanf(""), std::nanf(""), std::nanf("")}; // required for vel control in px4
	msg.velocity = {target_vel[0], target_vel[1], target_vel[2]};
	msg.yaw = 0.0; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint (position)
 */
void StarlingOffboard::publish_trajectory_setpoint_pos(const Eigen::Vector4f& target_pos)
{
	TrajectorySetpoint msg{};
	msg.position= {target_pos[0], target_pos[1], target_pos[2]};
	msg.yaw = 0.0; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish the trajectory setpoint (TwistStamped) to the PX4
 */
void StarlingOffboard::update_vel(const geometry_msgs::msg::TwistStamped::SharedPtr gnn_cmd_vel)
{
    // Proportional controller to maintain altitude
    const float kP = -1.0;
    const float err_z = (pos_msg_.z - takeoff_z);

    // Clamp the GNN velocity to [-1, 1], with mission scale factor
    // NOTE Flipping x and y to match the mission frame
    const Eigen::Vector4f vel_mission (
                               (float) clamp(scale * gnn_cmd_vel->twist.linear.y, -3.0, 3.0),
                               (float) clamp(scale * gnn_cmd_vel->twist.linear.x, -3.0, 3.0), 
                               -kP * err_z,
                               1.0);
    
    // Transform the velocity from the mission frame to NED
    vel_ned = tform(vel_mission, T_miss_ned);
    time_last_vel_update = clock_->now();
}

/**
 * @brief Publish the pose (PoseStamped) to the GNN
 */
void StarlingOffboard::publish_pose(const px4_msgs::msg::VehicleLocalPosition::SharedPtr vehicle_local_position)
{

    const Eigen::Vector4f vehicle_local_position_vec (vehicle_local_position->x, 
                                                      vehicle_local_position->y, 
                                                      vehicle_local_position->z, 
                                                      1.0);

    const Eigen::Vector4f vehicle_mission_position = tform(vehicle_local_position_vec, T_ned_miss);

    geometry_msgs::msg::PoseStamped gnn_pose;
    gnn_pose.header.stamp = this->get_clock()->now();

    // NOTE Flpping x and y to match the mission frame
    gnn_pose.pose.position.x = vehicle_mission_position[1];
    gnn_pose.pose.position.y = vehicle_mission_position[0];
    gnn_pose.pose.position.z = vehicle_mission_position[2];
    pose_publisher_->publish(gnn_pose);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * @param param5    Command parameter 5
 * @param param6    Command parameter 6
 * @param param7    Command parameter 7
 */
void StarlingOffboard::publish_vehicle_command(uint16_t command, float param1, float param2, double param5, double param6, float param7)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << " Eigen version : " << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl ;
 	std::cout << "Starting Starling offboard node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StarlingOffboard>());
	rclcpp::shutdown();
	return 0;
}
