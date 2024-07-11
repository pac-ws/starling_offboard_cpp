#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <sstream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum class State {
	IDLE,
	TAKEOFF,
	MISSION,
	LANDING
};

const unsigned char* toChar(State state){
	switch(state){
		case State::IDLE: return reinterpret_cast<const unsigned char*>("IDLE");
		case State::TAKEOFF: return reinterpret_cast<const unsigned char*>("TAKEOFF");
		case State::MISSION: return reinterpret_cast<const unsigned char*>("MISSION");
		case State::LANDING: return reinterpret_cast<const unsigned char*>("LANDING");
		default: return reinterpret_cast<const unsigned char*>("INVALID");
	}
}

std::ostream& operator<<(std::ostream& os, const unsigned char* state){
	return os << reinterpret_cast<const char*>(state);
}

std::ostream& operator<<(std::ostream& os, State state){
	return os << toChar(state);
}

struct Waypoint {
	float x;
	float y;
	float z;
};

struct DroneVel {
	float vx;
	float vy;
	float vz;
};


class StarlingOffboard: public rclcpp::Node
{
public:
	StarlingOffboard() : Node("offboard_control")
	{
        // Parameters
        this->declare_parameter("takeoff_altitude", 3.0);
        float takeoff_altitude = this->get_parameter("takeoff_altitude").as_float();
        
        this->declare_parameter("namespace", "rX");
        std::string ns = this->get_parameter("namespace").as_string();
        
        // QoS
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        // Pubs and Subs
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		pos_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg){
				pos_msg_ = *msg;
		});

		offboard_setpoint_counter_ = 0;

		// Set the home position for the local frame 
		set_home(LAT, LON, ALT);

		auto timer_callback = [this]() -> void {

			// arm
			// takeoff
			// loop through array of set points
			// 	if local position  within tolerance of waypoint
			// 		update waypoint
			//

			//std::ostringstream oss;
			//oss << "State: " << state_ << "\n";
			//std::string message = oss.str();
			//RCLCPP_INFO(this->get_logger(), message.c_str());

			switch (state_) {

				case State::IDLE:
				case State::TAKEOFF:
				case State::LANDING:

					//std::ostringstream oss;
					std::cout << "Version: " << 1.2 << "\n";
					//std::string message = oss.str();
					//RCLCPP_INFO(this->get_logger(), message.c_str());
					
					if (offboard_setpoint_counter_ == 10) {
						// Change to Offboard mode after 10 setpoints
						this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

						// Arm the vehicle
						this->arm();
						state_ = State::TAKEOFF;
					}
					
					// offboard_control_mode needs to be paired with trajectory_setpoint
					publish_offboard_control_mode(true, false);
					publish_trajectory_setpoint_pos(takeoff_pos);

					// error calculation
					takeoff = this->has_reached_pos(takeoff_pos);
					if (takeoff) {
						publish_trajectory_setpoint(stop_vel);
						state_ = State::MISSION;
						RCLCPP_INFO(this->get_logger(), "Takeoff complete -- reached setpoint within TOL");
					}

					// stop the counter after reaching 11
					if (offboard_setpoint_counter_ < 11) {
						offboard_setpoint_counter_++;
					}
					break;

				case State::MISSION:
					publish_offboard_control_mode(false, true);
					
					// compute the new velocity based on where the drone is wrt to the next waypoint
					DroneVel new_vel = compute_vel(waypts[way_pt_idx]);
					publish_trajectory_setpoint(new_vel);

					// error calculation
					waypt_reached = this->has_reached_pos(waypts[way_pt_idx]);
					if (waypt_reached) {
						way_pt_idx = (way_pt_idx + 1) % 4;
					}
					break;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_subscription_;
	px4_msgs::msg::VehicleLocalPosition pos_msg_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	bool takeoff = false;
	bool waypt_reached = false;

	// Waypoints are still used as goals
	// Fly to 5m at takeoff
	Waypoint takeoff_pos = {0.0, 0.0, -3.0};

	// 2m x 2m square at 3m altitude
	std::vector<Waypoint> waypts = { 
			{0.0, 0.0, -3.0},
			{2.0, 0.0, -3.0},
			{2.0, 2.0, -3.0},
			{0.0, 2.0, -3.0}
			};
	uint8_t way_pt_idx = 0;

	const float POS_TOL_ = 0.5; // waypoint position tolerance in meters
	const float VEL = 1.0;
	DroneVel stop_vel = {0.0, 0.0, 0.0};
	DroneVel takeoff_vel = {0.0, 0.0, -3.0};


	const double LAT =  39.94130201701814;
	const double LON = -75.19883699384171;
	const double ALT = 9.343585968017578;

	State state_ = State::IDLE;

	void publish_offboard_control_mode(const bool is_pos, const bool is_vel);
	void publish_trajectory_setpoint(const DroneVel& target_vel);
	void publish_trajectory_setpoint_pos(const Waypoint& target_pos);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, double param5 = 0.0, double param6 = 0.0, float param7 = 0.0);
	bool has_reached_pos(const Waypoint& target_pos);
	DroneVel compute_vel(const Waypoint& target_pos);
	void set_home(const double lat, const double lon, const float alt);
};

DroneVel StarlingOffboard::compute_vel(const Waypoint& target_pos){
	const float kP = 1.0;

	const float err_x = (pos_msg_.x - target_pos.x);
	const float err_y = (pos_msg_.y - target_pos.y);
	const float err_z = (pos_msg_.z - target_pos.z);

	DroneVel vel = {-kP * err_x, -kP * err_y, -kP * err_z};
	return vel;
}

bool StarlingOffboard::has_reached_pos(const Waypoint& target_pos)
{
	const float err_x = std::abs(pos_msg_.x - target_pos.x);
	const float err_y = std::abs(pos_msg_.y - target_pos.y);
	const float err_z = std::abs(pos_msg_.z - target_pos.z);

	std::ostringstream oss;
	oss << "ErrX = " << err_x << ", -- ErrY = " << err_y << ", -- ErrZ = " << err_z;
	std::string message = oss.str();

	RCLCPP_INFO(this->get_logger(), message.c_str());
	return err_x < POS_TOL_ && err_y < POS_TOL_ && err_z < POS_TOL_;
}

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
 * @brief Publish a trajectory setpoint
 */
void StarlingOffboard::publish_trajectory_setpoint(const DroneVel& target_vel)
{
	TrajectorySetpoint msg{};
	msg.position = {std::nanf(""), std::nanf(""), std::nanf("")}; // required for vel control in px4
	msg.velocity = {target_vel.vx, target_vel.vy, target_vel.vz};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void StarlingOffboard::publish_trajectory_setpoint_pos(const Waypoint& target_pos)
{
	TrajectorySetpoint msg{};
	msg.position= {target_pos.x, target_pos.y, target_pos.z};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
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
	std::cout << "Starting Starling offboard node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StarlingOffboard>());
	rclcpp::shutdown();
	return 0;
}
