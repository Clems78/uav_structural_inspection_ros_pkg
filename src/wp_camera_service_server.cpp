#include <memory>
#include <string>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <px4_msgs/msg/vehicle_local_position.hpp>


namespace fs = std::filesystem;

class WaypointCameraService : public rclcpp::Node
{
public:
  WaypointCameraService()
  : Node("wp_camera_service_server")
  {
    //Callback groups
    position_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    image_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    //Define options for subscribers
    rclcpp::SubscriptionOptions options_position;
    options_position.callback_group = position_callback_group_;

    rclcpp::SubscriptionOptions options_image;
    options_image.callback_group = image_callback_group_;

    // Define QoS settings to match publisher
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);



    // Publisher to send the trigger signal
    trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

    // Create a service to trigger the camera and save the image
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "capture_image", std::bind(&WaypointCameraService::capture_image_callback, this, std::placeholders::_1, std::placeholders::_2));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera", 10, std::bind(&WaypointCameraService::image_callback, this, std::placeholders::_1), options_image);

    position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&WaypointCameraService::local_position_callback, this, std::placeholders::_1), options_position);


    // Create directory if it doesn't exist
    fs::path save_dir = fs::path(getenv("HOME")) / "ros_saved_images";
    fs::create_directories(save_dir);
    save_dir_ = save_dir.string();

    fs::path save_dir_position = fs::path(getenv("HOME")) / "ros_saved_position";
    fs::create_directories(save_dir_position);
    save_dir_position_ = save_dir_position.string();

    // Open the mission file
    open_mission_file();

    //Variables
    this->image_received_ = false;
    this-> waypoint_id = 1;
  }

  ~WaypointCameraService()
  {
    // Close the mission file if it is open
    if (mission_file_.is_open())
    {
        mission_file_.close();
    }
  }

private:

  void open_mission_file()
  {
    // Get current date and time for the file name
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    auto datetime = oss.str();

    // File name
    std::string file_name = save_dir_position_ + "/mission_" + datetime + ".csv";

    // Open file in append mode
    mission_file_.open(file_name, std::ios_base::app);
    if (mission_file_.is_open())
    {
        // Write header if the file is new
        if (mission_file_.tellp() == 0)
        {
            mission_file_.setf(std::ios::fixed, std::ios::floatfield);
            mission_file_.precision(6);
            mission_file_ << "Waypoint,X,Y,Z,Orientation" << std::endl;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_name.c_str());
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
        // Convert the ROS image message to an OpenCV image using cv_bridge
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Create a filename for the image, incrementing the image_count_ each time
        std::ostringstream filename;
        filename << save_dir_ << "/image_" << image_count_++ << ".png";
        RCLCPP_INFO(this->get_logger(), "Image count : %ld", image_count_);


        // Save the OpenCV image to the specified filename
        cv::imwrite(filename.str(), cv_ptr->image);

        // Log a message indicating that the image was saved
        RCLCPP_INFO(this->get_logger(), "Saved image %s", filename.str().c_str());
        this->image_received_ = true;
    }
    catch (cv_bridge::Exception& e)
    {
        // If there is an error during conversion, log an error message
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void capture_image_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Publish the trigger signal
    std_msgs::msg::Bool trigger_msg;
    trigger_msg.data = true;
    trigger_pub_->publish(trigger_msg);
    RCLCPP_INFO(this->get_logger(), "Trigger signal sent");

    if (this->image_received_)
    {
      response->success = true;
      response->message = "Image captured and saved successfully.";
      this->image_received_ = false;  
    }
  }

  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {

    //std::cout << std::boolalpha;  // Enable textual representation of boolean values
    //std::cout << "Service done status: " << this->image_received_ << std::endl;

    //RCLCPP_INFO(this->get_logger(), "Waypoint ID : %d", waypoint_id);
    if (this->image_received_)
    {
        if (mission_file_.is_open() && waypoint_id == this->image_count_)
        {
            // Write data
            mission_file_ << "wp " << waypoint_id-1 << "," << msg->x << "," << msg->y << "," << msg->z << "," << msg->heading << std::endl;
              RCLCPP_INFO(this->get_logger(), "Viewpoints saved");


            // Increment waypoint_id for next waypoint
            waypoint_id++;
        }
    }
  }

  // Callback group
  rclcpp::CallbackGroup::SharedPtr position_callback_group_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;

  // Pub and Sub and Serv
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;

  // Variables
  bool image_received_; 
  std::string save_dir_;
  std::string save_dir_position_;
  size_t image_count_;
  int waypoint_id;
  std::ofstream mission_file_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<WaypointCameraService> waypoint_camera_service_node = std::make_shared<WaypointCameraService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(waypoint_camera_service_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
