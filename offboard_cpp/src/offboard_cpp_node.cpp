#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

// #include <px4_msgs/msg/timesync.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

using std::placeholders::_1;


class OffboardControl : public rclcpp::Node
{
public:
	explicit OffboardControl() : Node("offboard_control")
	{
    auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
	auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();


	// Create subscribers
	this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status",
		qos_sub, std::bind(&OffboardControl::vehicle_status_clbk, this, _1)
	);

	// // get common timestamp
	// this->timesync_sub_ =
	// 		this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", qos,
	// 			[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
	// 				timestamp_.store(msg->timestamp);
	// 			});

	// Create publishers
	this->offboard_control_mode_publisher_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_pub);
	this->trajectory_setpoint_publisher_ =
		this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_pub);
	this->vehicle_command_publisher_ =
		this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_pub);

	// Set default variables:
	this->offboard_setpoint_counter_ = 0;

	// timer callback
	auto timer_callback = [this]() -> void {

			if (this->offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            // offboard_control_mode needs to be paired with trajectory_setpoint
			this->publish_offboard_control_mode();
			this->publish_trajectory_setpoint();

           		 // stop the counter after reaching 11
			if (this->offboard_setpoint_counter_ < 11) {
				this->offboard_setpoint_counter_++;
			}
		};

	// start publisher timer
	timer_ = this->create_wall_timer(100ms, timer_callback);

	}

	void arm() ;
	void disarm() ;


private:

	// important node variables
	rclcpp::TimerBase::SharedPtr timer_;  // Timer that makes the node spin
	uint8_t nav_state;
	uint8_t arming_state;
	// std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent



	// Subscribers
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
		// rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;


	// publishers
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// callbacks

	void vehicle_status_clbk(const px4_msgs::msg::VehicleStatus & msg)
	{
		this->arming_state = msg.arming_state;
		this->nav_state = msg.nav_state;
		RCLCPP_INFO(this->get_logger(), "ArmingState: %i\nNavState: %i", this->arming_state, this->nav_state);
	}

	// commands
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint() ;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) ;

};

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2)  {
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}



/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()  {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()  {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()  {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()  {
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -M_PI; // [-PI:PI]

	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Entry point of Offboard node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char ** argv)
{
  std::cout << "Starting offboard_control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
