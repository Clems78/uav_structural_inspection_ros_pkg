#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        //Define callback group 
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        local_position_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        //Define options for subscribers
        rclcpp::SubscriptionOptions options_local_position;
        options_local_position.callback_group = local_position_callback_group_;

         // Define QoS settings to match publisher
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                       .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                       .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        
        //Publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        //Subscribers
        local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&OffboardControl::local_position_callback, this, std::placeholders::_1), options_local_position);

        // Define your list of waypoints here (latitude, longitude, altitude, yaw)
        waypoints_ = {
            {0, 0, -10.0, 0.0},  // Waypoint 1
            {10, 0, -10, 0.0},  // Waypoint 2
            {10, 10, -10, 0.0},  // Waypoint 3
            {0, 10, -10, 0.0},  // Waypoint 2
            {0, 0, -10, 0.0},  // Waypoint 2
            {10, 0, -10, 0.0},  // Waypoint 2
            {10, 10, -10, 0.0},  // Waypoint 3
            {0, 10, -10, 0.0},  // Waypoint 2
            {0, 0, -10, 0.0},  // Waypoint 2
            // Add more waypoints as needed...
        };

        this->current_waypoint_ = 0;
        offboard_setpoint_counter_ = 0;
        list_waypoints_reached_.assign(waypoints_.size(), false);


        auto timer_callback = [this]() -> void {

            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED, 1, 1, -1);
            }

            // OffboardControlMode needs to be paired with TrajectorySetpoint
            publish_offboard_control_mode();

            if (static_cast<std::size_t>(current_waypoint_) < waypoints_.size())            
            {
                publish_trajectory_setpoint(waypoints_[current_waypoint_]);

                if (this->waypoint_reached)
                {
                    this->current_waypoint_++;
                }
            } else
            {
                if (offboard_setpoint_counter_ == 11)
                {    
                    RCLCPP_INFO(this->get_logger(), "All waypoints reached");
                    land();
                    offboard_setpoint_counter_++;

                }

            }


            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }

            


        };
        timer_ = this->create_wall_timer(100ms, timer_callback,  timer_callback_group_);
    }

    void arm();
    void disarm();
    void land();

private:

void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "X: %f m / Y: %f m / Z: %f", msg->x, msg->y,  msg->z);
    std::vector<float> local_position = {msg->x, msg->y, msg->z};

    if (static_cast<std::size_t>(current_waypoint_) < waypoints_.size())    
        {
            this->waypoint_reached = check_waypoint_reached(this->waypoints_[this->current_waypoint_], local_position);
            if (this->waypoint_reached)
            {
                this->list_waypoints_reached_[current_waypoint_] = true;
            }
        }
}



    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr local_position_callback_group_;


    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;

    std::atomic<uint64_t> timestamp_; //!< common synced timestamped

    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
    int current_waypoint_;
    std::vector<std::vector<float>> waypoints_; //!< List of waypoints (latitude, longitude, altitude, yaw)

    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
    void publish_trajectory_setpoint(std::vector<float> waypoints);
    bool check_waypoint_reached(std::vector<float> waypoint, std::vector<float> local_position);
    bool waypoint_reached = false;
    std::vector<bool> list_waypoints_reached_;

};
/**
 * @brief Check if waypoint is reached 
 */
bool OffboardControl::check_waypoint_reached(std::vector<float> waypoint, std::vector<float> local_position)
{
    bool result = false;
    float delta_error = 1;

    float x_wp = waypoint[0];
    float y_wp = waypoint[1];
    float z_wp = waypoint[2];

    float x = local_position[0];
    float y = local_position[1];
    float z = local_position[2];

    float distance = sqrt(pow(x_wp - x, 2) + pow(y_wp - y, 2) + pow(z_wp - z, 2));

    //RCLCPP_INFO(this->get_logger(), "Distance to waypoint is: %f m", distance);


    if (distance <= delta_error && !list_waypoints_reached_[current_waypoint_])
    {
        result = true;
        RCLCPP_INFO(this->get_logger(), "Waypoint reached");
    } else if (distance <= delta_error)
    {
        result = true;
    }else
    {
        result = false;
    }


    return result;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Land command sent");
}


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}


void OffboardControl::publish_trajectory_setpoint(const std::vector<float> waypoints)
{
	TrajectorySetpoint msg{};
    msg.position = {waypoints[0], waypoints[1], waypoints[2]};
    msg.yaw = waypoints[3];
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

    //RCLCPP_INFO(this->get_logger(), "publishing waypoint");


}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * @param param3    Command parameter 3
 * @param param4    Command parameter 4
 * @param param5    Command parameter 5
 * @param param6    Command parameter 6
 * @param param7    Command parameter 7
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
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
    //RCLCPP_INFO(this->get_logger(), "Vehicle command sent: command=%d, params=[%f, %f, %f, %f, %f, %f, %f]",command, param1, param2, param3, param4, param5, param6, param7);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    std::shared_ptr<OffboardControl> offboard_control_node = std::make_shared<OffboardControl>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(offboard_control_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
