#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct Waypoint
{
    Waypoint()
        : Waypoint(0.0, 0.0) {}
    Waypoint(double x, double y)
        : x(x), y(y) {}
    Waypoint(std::vector<double> v)
        : x(v.at(0)), y(v.at(1)) {}

    double x;
    double y;
};

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();
    ~PurePursuit() {}

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;  

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::string map_frame_;
    double max_speed_;

    std::vector<Waypoint> waypoints_;
    Waypoint tracking_waypoint_;
    int closest_waypoint_idx_;

    tf2::Transform transform_ego_T_map_;

    // Visualtization variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wpt_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr track_wpt_marker_pub_;


    void poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void loadWaypoints();
    void updateClosestWaypoint(double look_ahead_dist, const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void updateTrackingPoint(double look_ahead_dist);
    double findSteeringAngle(double look_ahead_dist);
    void transformToEgoFrame(Waypoint &wpt);
    bool isBehindEgo(const Waypoint &wpt, const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);

    void timerCallback();
};
