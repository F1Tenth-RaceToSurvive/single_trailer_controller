
#include <sstream>

#include <string>
#include <cmath>
#include <vector>
#include "single_trailer_controller/save_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"



SaveWaypoints::SaveWaypoints()
    :Node("save_waypoints_node")
{

    std::cout << "Save Waypoints Node Started" << "\n";

    // Create a subscriber to the /goal_pose topic that waits for manual pose input from 2D goal pose in RViz
    pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
        std::bind(&SaveWaypoints::add_point_callback, this, std::placeholders::_1));

    // Get the file name as a ros param
    this->declare_parameter("file_name");
    this->declare_parameter("path_to_folder");

    // Store file path  
    file_name =  this->get_parameter("path_to_folder").as_string() + this->get_parameter("file_name").as_string();
    
    std::cout << "File name: " << file_name << "\n";

    std::cout << "Format of CSV is : position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w \n";
}


/**
 * @brief Read in the pose given by RViz goal and write it to a csv file
 * 
 * @param pose 
 */
void SaveWaypoints::add_point_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose){
    //Write to a csv file to my computer
    //Write to a file
    std::cout << "Adding point" << "\n";
    file.open(file_name, std::ios::app);
    file << pose->pose.position.x << "," << pose->pose.position.y << "," << pose->pose.position.z << "," << pose->pose.orientation.x << "," << pose->pose.orientation.y << "," << pose->pose.orientation.z << "," << pose->pose.orientation.w << "\n";
    file.close();
}



int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    std::shared_ptr<SaveWaypoints> node =  std::make_shared<SaveWaypoints> ();
    rclcpp::spin(node);
    rclcpp::shutdown();
    std::cout << "Save Waypoints Node Ended" << "\n";
    return 0;
}