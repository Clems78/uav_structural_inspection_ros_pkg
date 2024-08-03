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
#include <geometry_msgs/msg/pose_array.hpp>

#include <nav_msgs/msg/odometry.hpp>




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
    gt_pose_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gt_vision_pose_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


    //Define options for subscribers
    rclcpp::SubscriptionOptions options_position;
    options_position.callback_group = position_callback_group_;

    rclcpp::SubscriptionOptions options_image;
    options_image.callback_group = image_callback_group_;

    rclcpp::SubscriptionOptions options_gt_pose;
    options_gt_pose.callback_group = gt_pose_callback_group_;

    rclcpp::SubscriptionOptions options_gt_vision_pose;
    options_gt_vision_pose.callback_group = gt_vision_pose_callback_group_;

    // Define QoS settings to match publisher
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Publisher to send the trigger signal
    trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

    // Create a service to trigger the camera and save the image
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "capture_image", std::bind(&WaypointCameraService::capture_image_callback, this, std::placeholders::_1, std::placeholders::_2));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera", 10, std::bind(&WaypointCameraService::image_callback, this, std::placeholders::_1), options_image);

    position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&WaypointCameraService::local_position_callback, this, std::placeholders::_1), options_position);

    //gt_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/world/irp/pose/info", 10, std::bind(&WaypointCameraService::gt_pose_callback, this, std::placeholders::_1), options_gt_pose);
    gt_vision_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/x500_vision_0/odometry_with_covariance", qos_2, std::bind(&WaypointCameraService::gt_vision_pose_callback, this, std::placeholders::_1), options_gt_vision_pose);

    // Create directory if it doesn't exist
    fs::path save_dir = fs::path(getenv("HOME")) / "ros_saved_images";
    fs::create_directories(save_dir);
    save_dir_ = save_dir.string();

    fs::path save_dir_position = fs::path(getenv("HOME")) / "ros_saved_position";
    fs::create_directories(save_dir_position);
    save_dir_position_ = save_dir_position.string();

    // Open the mission file
    open_estimated_pose_file();
    open_gt_pose_file();


    //Variables
    this->image_received_ = false;
    this-> waypoint_id = 1;
    this-> waypoint_id_gt = 1;
  }

  ~WaypointCameraService()
  {
    // Close the mission file if it is open
    if (estimated_pose_file_.is_open())
    {
        estimated_pose_file_.close();
    }
        // Close the mission file if it is open
    if (gt_pose_file_.is_open())
    {
        gt_pose_file_.close();
    }
  }

private:

  void open_estimated_pose_file()
  {
    // Get current date and time for the file name
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    auto datetime = oss.str();

    // File name
    std::string file_name = save_dir_position_ + "/estimated_pose_" + datetime + ".csv";

    // Open file in append mode
    estimated_pose_file_.open(file_name, std::ios_base::app);
    if (estimated_pose_file_.is_open())
    {
        // Write header if the file is new
        if (estimated_pose_file_.tellp() == 0)
        {
            estimated_pose_file_.setf(std::ios::fixed, std::ios::floatfield);
            estimated_pose_file_.precision(6);
            estimated_pose_file_ << "Waypoint,X,Y,Z,Orientation" << std::endl;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_name.c_str());
    }
  }

    void open_gt_pose_file()
  {
    // Get current date and time for the file name
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    auto datetime = oss.str();

    // File name
    std::string file_name = save_dir_position_ + "/gt_pose_" + datetime + ".csv";

    // Open file in append mode
    gt_pose_file_.open(file_name, std::ios_base::app);
    if (gt_pose_file_.is_open())
    {
        // Write header if the file is new
        if (gt_pose_file_.tellp() == 0)
        {
            gt_pose_file_.setf(std::ios::fixed, std::ios::floatfield);
            gt_pose_file_.precision(6);
            gt_pose_file_ << "Waypoint,X pose,Y pose,Z pose,X orientation,Y orientation,Z orientation,W orientation" << std::endl;
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
    if (this->image_received_)
    {
      if (estimated_pose_file_.is_open() && waypoint_id == this->image_count_)
      {
        // Write data
        estimated_pose_file_ << "wp " << waypoint_id-1 << "," << msg->x << "," << msg->y << "," << msg->z << "," << msg->heading << std::endl;
          RCLCPP_INFO(this->get_logger(), "Estimated pose saved");

        // Increment waypoint_id for next waypoint
        waypoint_id++;
      }
    }
  }

  // void gt_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  // {
  //   size_t x500_index = 2;
  //   auto pose = msg->poses[x500_index];
  //   bool selected = false;
  //   //RCLCPP_INFO(this->get_logger(), "GT: X: %f m / Y: %f m / Z: %f", pose.position.x, pose.position.y,  pose.position.z);


  //   if (this->image_received_ && selected == true)
  //   {
  //     if (gt_pose_file_.is_open() && waypoint_id_gt == this->image_count_)
  //     {
  //       // Write data (x and y switched to matct matlab and localvehicle setpoint frame )
  //       gt_pose_file_ << "wp " << waypoint_id_gt-1 << "," << pose.position.y << "," << pose.position.x << "," << pose.position.z << "," << pose.orientation.y << "," << pose.orientation.x << "," << pose.orientation.z << "," << pose.orientation.w << std::endl;
  //       RCLCPP_INFO(this->get_logger(), "Ground truth pose saved");


  //       // Increment waypoint_id for next waypoint
  //       waypoint_id_gt++;
  //     }
  //   }
  // }


  void gt_vision_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    //auto pose = msg->poses[x500_index];
    RCLCPP_INFO(this->get_logger(), "GT: X: %f m / Y: %f m / Z: %f", msg->pose.pose.position.y, msg->pose.pose.position.x,  msg->pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Image count %d", this->image_count_);

    if (this->image_received_)
    {
      if (gt_pose_file_.is_open() && waypoint_id_gt == this->image_count_)
      {
        // Write data (x and y switched to matct matlab and localvehicle setpoint frame )
        gt_pose_file_ << "wp " << waypoint_id_gt-1 << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.x << "," << msg->pose.pose.position.z << "," << msg->pose.pose.orientation.y << "," << msg->pose.pose.orientation.x << "," << msg->pose.pose.orientation.z << "," << msg->pose.pose.orientation.w << std::endl;
        RCLCPP_INFO(this->get_logger(), "Ground truth pose saved");

        // Increment waypoint_id for next waypoint
        waypoint_id_gt++;
      }
    }
  }

  // Callback group
  rclcpp::CallbackGroup::SharedPtr position_callback_group_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr gt_pose_callback_group_;
  rclcpp::CallbackGroup::SharedPtr gt_vision_pose_callback_group_;



  // Pub and Sub and Serv
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  //rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gt_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_vision_pose_sub_;



  // Variables
  bool image_received_; 
  std::string save_dir_;
  std::string save_dir_position_;
  size_t image_count_;
  int waypoint_id;
  int waypoint_id_gt;
  std::ofstream estimated_pose_file_;
  std::ofstream gt_pose_file_;
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
