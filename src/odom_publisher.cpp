#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <array>
#include <mavros_msgs/msg/companion_process_status.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include <chrono>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("odom_publisher_node")
    {
        //Define callback group 
        odom_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        //Define options for subscribers
        rclcpp::SubscriptionOptions options_odom;
        options_odom.callback_group = odom_callback_group_;

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        //Publishers
        odom_publisher_px4_ = this->create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/mavros/odometry/out", 10);
        comp_status_publisher_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>("/mavros/companion_process/status", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&OdomPublisher::timer_callback, this),  timer_callback_group_);

        //Subscribers
        odom_position_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/x500_vision_0/odometry_with_covariance", qos, std::bind(&OdomPublisher::odom_callback, this, std::placeholders::_1), options_odom);
           
        // Publish Static Transform
        publish_static_transform();
    }

private:

    void timer_callback()
    {
        auto msg = mavros_msgs::msg::CompanionProcessStatus();
        msg.header.stamp = this->now();
        msg.component = msg.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
        msg.state = msg.MAV_STATE_ACTIVE;

        comp_status_publisher_->publish(msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {   

        //uint64_t timestamp_odom = (msg->header.stamp.sec)/1e3;
        //uint8_t pose_frame_odom = 1;

        //uint8_t velocity_frame_odom = 0;
        bool mavros = true;

        if (mavros == false)
        {
            VehicleOdometry odom_msg{};

            std::array<float, 3> position_odom = {
                static_cast<float>(msg->pose.pose.position.x),
                static_cast<float>(msg->pose.pose.position.y),
                static_cast<float>(msg->pose.pose.position.z)};
                
            std::array<float, 4> q_odom = {
                static_cast<float>(msg->pose.pose.orientation.x),
                static_cast<float>(msg->pose.pose.orientation.y),
                static_cast<float>(msg->pose.pose.orientation.z),
                static_cast<float>(msg->pose.pose.orientation.w)};

            std::array<float, 3> velocity_odom = {
                static_cast<float>(msg->twist.twist.linear.x),
                static_cast<float>(msg->twist.twist.linear.y),
                static_cast<float>(msg->twist.twist.linear.z)};

            std::array<float, 3> angular_velocity_odom = {
                static_cast<float>(msg->twist.twist.angular.x),
                static_cast<float>(msg->twist.twist.angular.y), 
                static_cast<float>(msg->twist.twist.angular.z)};

            std::array<float, 3> position_variance_odom = {0.0f, 0.0f, 0.0f};
            std::array<float, 3> orientation_variance_odom = {0.0f, 0.0f, 0.0f};
            std::array<float, 3> velocity_variance_odom = {0.0f, 0.0f, 0.0f};

            //create the message
            odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            //odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
            odom_msg.position = position_odom;
            odom_msg.q = q_odom;
            //odom_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
            odom_msg.velocity = velocity_odom;
            odom_msg.angular_velocity = angular_velocity_odom;
            odom_msg.position_variance = position_variance_odom;
            odom_msg.orientation_variance = orientation_variance_odom;
            odom_msg.velocity_variance = velocity_variance_odom;
            odom_msg.quality = 1;

            odom_publisher_px4_->publish(odom_msg);
        }
        else
        {
            odom_publisher_->publish(*msg);
        }
    }

    void publish_static_transform()
    {
        static tf2_ros::StaticTransformBroadcaster static_broadcaster(this);
        geometry_msgs::msg::TransformStamped static_transform_stamped;

        static_transform_stamped.header.stamp = this->now();
        static_transform_stamped.header.frame_id = "x500_vision_0/odom";
        static_transform_stamped.child_frame_id = "x500_vision_0/odom_ned";
        static_transform_stamped.transform.translation.x = 0.0;
        static_transform_stamped.transform.translation.y = 0.0;
        static_transform_stamped.transform.translation.z = 0.0;
        static_transform_stamped.transform.rotation.x = 0.0;
        static_transform_stamped.transform.rotation.y = 0.0;
        static_transform_stamped.transform.rotation.z = 0.0;
        static_transform_stamped.transform.rotation.w = 1.0;

        static_broadcaster.sendTransform(static_transform_stamped);
    }

    // Callback group
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    // Timer / sub / pub / serv
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_publisher_px4_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr comp_status_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_position_subscription_;

    // Function 

    // Variables

};

int main(int argc, char *argv[])
{
    std::cout << "Starting odom publisher node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    std::shared_ptr<OdomPublisher> node = std::make_shared<OdomPublisher>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
