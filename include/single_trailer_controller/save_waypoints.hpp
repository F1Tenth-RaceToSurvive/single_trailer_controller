#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <fstream>



class SaveWaypoints : public rclcpp::Node
{
public:
    SaveWaypoints();
    std::ofstream file;
    std::string file_name;

private:
    
    // laserscan subscriber and ackermann drive publisher
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
    
    
    void add_point_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose);
};
