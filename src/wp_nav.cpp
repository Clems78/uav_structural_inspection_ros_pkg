#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>

#include <chrono>
#include <iostream>
#include <fstream>   // Required for std::ifstream
#include <string>
#include <sstream>

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

        // Define service client
        client_ = this->create_client<std_srvs::srv::Trigger>("capture_image");

        
        //Publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        //Subscribers
        local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&OffboardControl::local_position_callback, this, std::placeholders::_1), options_local_position);
       
        // Load waypoints from file
        // Hardcoded file path
        file_path_ = "/home/clem/waypoints_ros.txt";
        load_waypoints_from_file(file_path_);

        // Define your list of waypoints here (latitude, longitude, altitude, yaw)
        /*
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
        };*/

        // waypoints_ = 
        // {   
        //     {-0, 0, -3, 90},
        //     {-0.8021, 1.1067, -2.8488, 90},
        //     {-1.9020, 1.1067, -2.7909, 90},
        //     {-1.9020, 1.1067, -4.0612, 90},
        //     {-1.8021, 1.1067, -5.3891, 90},
        //     {-1.9012, 1.1067, -6.6002, 90},
        //     {-0.8010, 1.1067, -6.6555, 90},
        //     {0.4949, 1.1067, -6.5982, 90},
        //     {1.7974, 1.1067, -6.6553, 90},
        //     {1.9936, 1.1067, -5.6161, 90},
        //     {0.7979, 1.1067, -5.2736, 90},
        //     {-0.5021, 1.1067, -5.4468, 90},
        //     {-0.5021, 1.1067, -4.1767, 90},
        //     {0.7979, 1.1067, -4.0035, 90},
        //     {1.9963, 1.1067, -4.2339, 90},
        //     {1.7979, 1.1067, -2.8488, 90},
        //     {0.4979, 1.1067, -2.7910, 90},
        //     {-0, 0, -3, 90}
        // };

       /* waypoints_ = 
        {   
            {0, 0, -3, 90},
            {0.8021, 1.1067, -2.8488, 90},
            {1.9020, 1.1067, -2.7909, 90},
            {1.9020, 1.1067, -4.0612, 90},
            {0.5021, 1.1067, -4.1767, 90},
            {0.5021, 1.1067, -5.4468, 90},
            {1.8021, 1.1067, -5.3891, 90},
            {1.9012, 1.1067, -6.6002, 90},
            {0.8010, 1.1067, -6.6555, 90},
            {-0.4949, 1.1067, -6.5982, 90},
            {-1.7974, 1.1067, -6.6553, 90},
            {-1.9936, 1.1067, -5.6161, 90},
            {-0.7979, 1.1067, -5.2736, 90},
            {-0.7979, 1.1067, -4.0035, 90},
            {-1.9963, 1.1067, -4.2339, 90},
            {-1.7979, 1.1067, -2.8488, 90},
            {-0.4979, 1.1067, -2.7910, 90},
            {0, 0, -3, 90},
        };*/

      /*  waypoints_ = 
{   
    {0.000000, 0.000000, -3.000000, 90.000000},
    {3.202973, 2.061287, -5.426722, -163.585669},
    {4.171244, 4.015697, -6.313982, 11.451390},
    {4.169587, 4.007616, -7.269048, -91.180720},
    {4.178064, 4.050165, -8.217822, 114.135129},
    {3.837413, 3.025188, -9.186965, -173.641222},
    {2.799296, 8.102297, -9.427508, 11.449738},
    {2.875895, 8.034802, -5.274360, -174.995143},
    {-0.134678, 9.167949, -6.241913, 11.909695},
    {-0.096890, 9.168680, -7.200262, -105.647375},
    {-1.166410, 9.015976, -8.153139, 114.261873},
    {-3.086694, 7.890169, -9.127003, -91.113082},
    {-3.697599, 7.101848, -5.441849, 114.115177},
    {-4.291271, 5.365134, -6.284848, -135.207472},
    {-4.146781, 6.087020, -7.382115, 41.369292},
    {-4.305832, 5.244985, -8.226882, -149.164933},
    {-0.773091, 0.655660, -9.186088, 25.908099},
    {-1.801917, 0.973861, -8.226417, 99.691149},
    {-1.764408, 0.957055, -7.270929, -47.064894},
    {-1.811658, 0.978247, -6.311987, -49.108335},
    {-2.669054, 1.498699, -4.950245, 128.214700},
    {0.000000, 0.000000, -3.000000, 90.000000}
};*/





        //for (size_t i = 0; i < waypoints_.size(); ++i)
        //{
        //RCLCPP_INFO(this->get_logger(), "WP %f : X : %f, Y : %f, Z: %f, Yaw: %f",i, waypoints_[i][0], waypoints_[i][1], waypoints_[i][2], waypoints_[i][3]);
        //}

        this->current_waypoint_ = 0;
        offboard_setpoint_counter_ = 0;
        list_waypoints_reached_.assign(waypoints_.size(), false);
        list_waypoints_reached_global_.assign(waypoints_.size(), false);



        auto timer_callback = [this]() -> void {

            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED, 1, 1);
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 47.397971057728974, 8.546163739800146, 0);

            }

            // OffboardControlMode needs to be paired with TrajectorySetpoint
            publish_offboard_control_mode();

            if (static_cast<std::size_t>(current_waypoint_) < waypoints_.size())            
            {
                publish_trajectory_setpoint(waypoints_[current_waypoint_]);
                RCLCPP_INFO(this->get_logger(), "WP %d : X : %f, Y : %f, Z: %f, Yaw: %f", current_waypoint_, 
                waypoints_[current_waypoint_][0], waypoints_[current_waypoint_][1], waypoints_[current_waypoint_][2], waypoints_[current_waypoint_][3]);

                if (this->waypoint_reached && this->flag_) 
                {

                    this->current_waypoint_++;
                    RCLCPP_INFO(this->get_logger(), "current waypoint: %d",current_waypoint_);
                    this->flag_ = false;
      
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

 void load_waypoints_from_file(const std::string &file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening file: %s", file_path.c_str());
            return;
        }

        waypoints_.clear();
        std::string line;
        while (std::getline(file, line))
        {
            std::vector<float> waypoint;
            std::stringstream ss(line);
            float value;
            while (ss >> value)
            {
                waypoint.push_back(value);
                if (ss.peek() == ' ' || ss.peek() == '\t')
                    ss.ignore();
            }
            if (waypoint.size() == 4) // Ensure there are exactly 4 values: x, y, z, heading
            {
                waypoints_.push_back(waypoint);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Skipping malformed line: %s", line.c_str());
            }
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from file: %s", waypoints_.size(), file_path.c_str());
        for (size_t i = 0; i < waypoints_.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "WP : X : %f, Y : %f, Z: %f, Yaw: %f", 
                        waypoints_[i][0], waypoints_[i][1], waypoints_[i][2], waypoints_[i][3]);
        }
    }

void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{   
    //RCLCPP_INFO(this->get_logger(), "X: %f m / Y: %f m / Z: %f", msg->x, msg->y,  msg->z);
    std::vector<float> local_position = {msg->x, msg->y, msg->z, msg->heading};

    if (static_cast<std::size_t>(current_waypoint_) < waypoints_.size())    
        {
            this->waypoint_reached = check_waypoint_reached(this->waypoints_[this->current_waypoint_], local_position);
            if (this->waypoint_reached && !list_waypoints_reached_[current_waypoint_])
            {
                this->list_waypoints_reached_[current_waypoint_] = true;
                this->flag_ = true;                
            }
        }
}

    // Callback group
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr local_position_callback_group_;

    // Timer / sub / pub / serv
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;

    // Function 
    void publish_offboard_control_mode();
    void publish_vehicle_command(uint32_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
    void publish_trajectory_setpoint(std::vector<float> waypoints);
    bool check_waypoint_reached(std::vector<float> waypoint, std::vector<float> local_position);
    void capture_image_call();
    void response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
    
    // Variables
    std::atomic<uint64_t> timestamp_; //!< common synced timestamped
    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
    int current_waypoint_;
    std::vector<std::vector<float>> waypoints_; //!< List of waypoints (latitude, longitude, altitude, yaw)
    bool waypoint_reached = false;
    std::vector<bool> list_waypoints_reached_;
    std::vector<bool> list_waypoints_reached_global_;
    bool service_done_ = false;
    int capture_image_counter_ = 0;
    std::string file_path_; 
    bool flag_ = false;
};
/**
 * @brief Check if waypoint is reached 
 */
bool OffboardControl::check_waypoint_reached(std::vector<float> waypoint, std::vector<float> local_position)
{
    bool result = false;
    float delta_pose_error = 0.2;
    float delta_heading_error = 1; // degree 

    float x_wp = waypoint[0];
    float y_wp = waypoint[1];
    float z_wp = waypoint[2];
    float heading_wp = waypoint[3]; //degree

    float x = local_position[0];
    float y = local_position[1];
    float z = local_position[2];
    float heading = local_position[3]; // rad

    float heading_error = abs(heading_wp - (heading * 180 / M_PI)); //degree
    float distance = sqrt(pow(x_wp - x, 2) + pow(y_wp - y, 2) + pow(z_wp - z, 2));

    //RCLCPP_INFO(this->get_logger(), "Distance error : %f m", distance);
    //RCLCPP_INFO(this->get_logger(), "Heading error : %f degree", heading_error);

    if (distance <= delta_pose_error && !list_waypoints_reached_[current_waypoint_] && (heading_error <= delta_heading_error))
    {
        result = true;
        capture_image_call();
        RCLCPP_INFO(this->get_logger(), "Waypoint %d reached", current_waypoint_);
        //this-> wp_flag_ = true;

        //std::cout << std::boolalpha;  // Enable textual representation of boolean values
        //std::cout << "Service done status: " << service_done_ << std::endl;

    } else if (distance <= delta_pose_error && (heading_error <= delta_heading_error))
    {
        result = true;
    }else
    {
        result = false;
        service_done_ = false;
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
    msg.yaw = waypoints[3] * M_PI / 180; //degree to radian
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
void OffboardControl::publish_vehicle_command(uint32_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
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

void OffboardControl::capture_image_call()
{
    while (!client_->wait_for_service(500ms))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service. Terminating...");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service unavailable. Waiting for service..");
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    service_done_ = false;
    auto result_future = client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this, std::placeholders::_1));
 }

void OffboardControl::response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) 
{
    auto status = future.wait_for(250ms);
    if (status == std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "Image and location capture : Success");
        service_done_ = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

//bool OffboardControl::is_service_done() const { return this->service_done_; }

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
