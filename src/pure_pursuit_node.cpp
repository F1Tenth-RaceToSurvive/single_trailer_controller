#include <sstream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "single_trailer_controller/pure_pursuit_node.hpp"


/**
 * @brief Construct a new Pure Pursuit:: Pure Pursuit object
 *
 */
PurePursuit::PurePursuit()
    : Node("pure_pursuit_node"), closest_waypoint_idx_(-1)
{
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);


    // Declare ROS parameters
    this->declare_parameter("enable_vis");
    this->declare_parameter("map_frame");
    this->declare_parameter("drive_topic");
    this->declare_parameter("pose_topic");
    this->declare_parameter("waypoint_filename");
    this->declare_parameter("look_ahead_dist");
    this->declare_parameter("window_size");
    this->declare_parameter("viable_dist");
    this->declare_parameter("steering_gain");
    this->declare_parameter("max_speed");

    map_frame_ = this->get_parameter("map_frame").as_string();

    // Ros publishers and subscribers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        this->get_parameter("drive_topic").as_string(), 1);
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("pose_topic").as_string(),
        1,
        std::bind(&PurePursuit::poseCallback, this, std::placeholders::_1));
    

    
    if(this->get_parameter("enable_vis").as_bool() == true)
    {
        this->declare_parameter("waypoint_vis_topic");
        this->declare_parameter("track_waypoint_vis_topic");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PurePursuit::timerCallback, this));         // Publish vis at 10 hz
        wpt_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(this->get_parameter("waypoint_vis_topic").as_string(), 1);
        track_wpt_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(this->get_parameter("track_waypoint_vis_topic").as_string(), 1);
    }

    // This loads the waypoints from a csv file containing the information in the following format:
    // x, y, z, q.x, q.y, q.z, q.w : x, y, z are the position of the waypoint and q.x, q.y, q.z, q.w are the orientation of the waypoint
    loadWaypoints();
}

/**
 * @brief Determines the closest waypoint in front of the car and updates the index to the member variable closest_waypoint_idx_
 *        It searches for within a window of the last waypoint assuming that the car cannot move too far from the last waypoint it was close to
 * 
 * @param look_ahead_dist The distance beyond which the car will look for the closest waypoint
 * @param pose_msg The current pose of the car
 *
*/
void PurePursuit::updateClosestWaypoint(double look_ahead_dist, const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{

    // Determines how many waypoints to search through starting from the last closest waypoint
    int window_size = this->get_parameter("window_size").as_int();

    // distance beyond which the search over the window is is defintely incorrect
    double viable_dist = this->get_parameter("viable_dist").as_double();



    // Find closest waypoint in front of the car
    double closest_dist = DBL_MAX;
    int closest_idx;
    double dist;
    double car_pos_x = pose_msg->pose.pose.position.x;
    double car_pos_y = pose_msg->pose.pose.position.y;

    // If we have an estimate of the last closest waypoint, search through a window of waypoints around it
    //Otherwise, search through the entire list of waypoints
    // int start_idx = closest_waypoint_idx_ == -1 ? 0 : closest_waypoint_idx_;
    // int end_idx = closest_waypoint_idx_ == -1 ? waypoints_.size() : closest_waypoint_idx_ + window_size;

    int start_idx = 0;
    int end_idx = waypoints_.size();

    for(int i = start_idx; i < end_idx; i++)
    {
        int idx = i%(waypoints_.size());
        dist = sqrt(pow((car_pos_x - waypoints_.at(idx).x), 2) + pow(car_pos_y - waypoints_.at(idx).y, 2));
        
        // COnsider point only if it is in front of the car and closer than the current closest point, and beyond the look ahead distance
        if(dist >= look_ahead_dist && dist < closest_dist && !(isBehindEgo(waypoints_.at(idx), pose_msg)))
        {
            closest_dist = dist;
            closest_idx = idx;
        }
    }
    
    // Reset closest_waypoint_idx_ to -1 to trigger searching through the full waypoint list if the closest waypoint is too far away to be true
    closest_waypoint_idx_ = closest_dist > viable_dist ? -1 : closest_idx;
}

/**
 * @brief Determines the tracking waypoint based on the closest waypoint in front of the car and the look ahead distance
 * 
 * @param look_ahead_dist The distance at which the car will determine a point to track
 * 
 * @return Waypoint The tracking waypoint in the ego frame
*/
void PurePursuit::updateTrackingPoint(double look_ahead_dist){

    // Draw a line from the current waypoint and the previous point
    Waypoint current = waypoints_.at(closest_waypoint_idx_);
    Waypoint prev = waypoints_.at((closest_waypoint_idx_ -1 + waypoints_.size()) % waypoints_.size()); // The previous index accounting with rollover
    

    // Transform these points to the ego frame
    transformToEgoFrame(current);
    transformToEgoFrame(prev);

    // Find where the line intersects with the circle of radius look_ahead_dist
    double m = (current.y - prev.y)/(current.x - prev.x);
    double c = current.y - m*current.x;

    double a_poly = 1 + pow(m,2);
    double b_poly = 2*m*c;
    double c_poly =  pow(c,2) - pow(look_ahead_dist,2);

    double disc = pow(b_poly,2) - 4*a_poly*c_poly ;

    if(disc > 0)
    {
        // If the line intersects with the arc, the roots are real
        
        double x_root1 = (-b_poly + sqrt(disc))/(2*a_poly);
        double x_root2 = (-b_poly - sqrt(disc))/(2*a_poly);

        double y_root1 = m*x_root1 + c;
        double y_root2 = m*x_root2 + c;

        double dist1 = sqrt(pow((current.x - x_root1), 2) + pow(current.y - y_root1, 2));
        double dist2 = sqrt(pow((current.x - x_root2), 2) + pow(current.y - y_root2, 2));


        // If the roots are real, the tracking point is the one that is closer to the current point
        if(dist1 < dist2)
        {
            tracking_waypoint_.x = x_root1;
            tracking_waypoint_.y = y_root1;
        }
        else
        {
            tracking_waypoint_.x = x_root2;
            tracking_waypoint_.y = y_root2;
        }
    }
    else if(disc == 0)
    {
        //Roots are equal
        double x_root = -b_poly/(2*a_poly);
        double y_root = m* x_root + c;

        tracking_waypoint_.x = x_root;
        tracking_waypoint_.y = y_root;
    }
    else
    {
        //Imaginary roots case : track the point on the arc that is a project of the cloest point on the line
        double dist_body_to_tracking = sqrt(pow(current.x, 2) + pow(current.y, 2));
        tracking_waypoint_.x = look_ahead_dist*(current.x/dist_body_to_tracking);
        tracking_waypoint_.y = look_ahead_dist*(current.y/dist_body_to_tracking);
    }  
}

/**
 * @brief Determines the steering angle based on the tracking waypoint and the look ahead distance : Uses <formula>
 * 
 * @param look_ahead_dist The distance at which the the tracking point is
 * 
 * @return double The steering angle in radians
*/
double PurePursuit::findSteeringAngle(double look_ahead_dist)
{
    double steering_gain = this->get_parameter("steering_gain").as_double();

    double curvature = (2*abs(tracking_waypoint_.y))/pow(look_ahead_dist, 2);

    double steering_angle = steering_gain * curvature;
    
    if(tracking_waypoint_.y < 0)
        steering_angle = -steering_angle;

    return steering_angle;    
}

/**
 * @brief Each time we get odom information, we determine the steering control and publish command to the car
 * 
 * @param pose_msg The pose message of the car from the particle filter/sim
*/
void PurePursuit::poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose_msg->pose.pose.orientation, orientation);
    tf2::Vector3 translation(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, 0.0);

    transform_ego_T_map_ = tf2::Transform(orientation, translation).inverse();

    double look_ahead_dist = this->get_parameter("look_ahead_dist").as_double();
    
    updateClosestWaypoint(look_ahead_dist, pose_msg);

    if(closest_waypoint_idx_ == -1)
    {
        std::cout << "The Robot is Lost!" << std::endl;
        return;    
    }
    max_speed_ = this->get_parameter("max_speed").as_double();

    updateTrackingPoint(look_ahead_dist);
    double steering_angle = findSteeringAngle(look_ahead_dist);

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = max_speed_;    //TODO: maybe add a velocity for each waypoint?

    drive_pub_->publish(drive_msg); //TODO: Think about capping steering angle value
}

/**
 * @brief Loads the waypoints from a csv file to the member variable waypoints_
 *        Currently only loads the first two components of the waypoint (x,y)
 * 
 * */
void PurePursuit::loadWaypoints()
{
    std::string package_root_dir = ament_index_cpp::get_package_share_directory("single_trailer_controller");
    std::string waypoint_file_path = package_root_dir + "/waypoints/" + this->get_parameter("waypoint_filename").as_string();
    std::ifstream waypoint_file(waypoint_file_path);
    std::string line;

    while (getline (waypoint_file, line)) 
    {
        std::stringstream s_stream(line); 
        std::string substr;
        std::vector<double> waypoint_components;

        // Just reads the first two components of the waypoint (x,y)
        for(int i = 0; i < 2; i++)
        {
            getline(s_stream, substr, ','); 
            waypoint_components.push_back(std::stod(substr));
        }

        waypoints_.push_back(Waypoint(waypoint_components));
    }
    waypoint_file.close();
}

/**
 * @brief Transforms the waypoint from the map frame to the ego frame
 * 
*/
void PurePursuit::transformToEgoFrame(Waypoint &wpt)
{
    tf2::Vector3 pt(wpt.x, wpt.y, 0.0);

    try
    {
        pt = transform_ego_T_map_ * pt;
        wpt.x = pt.getX();
        wpt.y = pt.getY();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

/**
 * @brief Checks if the waypoint is behind the ego vehicle
 * 
 * @param wpt The waypoint to check
 * @param pose_msg The pose message of the ego vehicle
 * 
 * @return true If the waypoint is behind the ego vehicle
*/
bool PurePursuit::isBehindEgo(const Waypoint &wpt, const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{
    tf2::Quaternion ego_orientation;
    tf2::fromMsg(pose_msg->pose.pose.orientation, ego_orientation);    // Set the quaternion message to tf2 quaternion object for manipulation

    tf2::Vector3 ego_direction(1, 0, 0);  // Intialize the forward facing direction of ego in the map frame
    ego_direction = tf2::quatRotate(ego_orientation, ego_direction);    // Rotate intial direction by the actual orientation of ego

    // Construct another vector that indicates the direction the waypoint is with respect to ego
    tf2::Vector3 ego_to_wpt_direction(wpt.x - pose_msg->pose.pose.position.x,
                                      wpt.y - pose_msg->pose.pose.position.y,
                                      0.0);

    /*  To check if the waypoint is behind ego we just need to compare we just need to compare if the vector defining
        the forward facing direction of ego and the vector defining the direction the waypoint is with respect to
        ego are facing in relatively the same geometric direction. We can do this by taking the dot product of the two
        vectors. A positve result means they are facing in the same direction and negative means otherwise.
    */
    if(tf2::tf2Dot(ego_direction, ego_to_wpt_direction) < 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Periodic callback function to publish the visualization messages
 *
 */
void PurePursuit::timerCallback()
{
    /*
        Waypoint List Visualization
    */
    visualization_msgs::msg::Marker wpt_marker;
    wpt_marker.header.frame_id = map_frame_;
    wpt_marker.header.stamp = this->get_clock()->now();

    wpt_marker.id = 0;      // This needs to be a unique number per marker
    wpt_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    for(const Waypoint& wpt : waypoints_)
    {
        geometry_msgs::msg::Point p;
        p.x = wpt.x;
        p.y = wpt.y;
        p.z = 0.0;
        wpt_marker.points.push_back(p);
    }
    wpt_marker.scale.x = 0.2;
    wpt_marker.scale.y = 0.2;
    wpt_marker.scale.z = 0.2;

    // Color values must be [0, 1]
    wpt_marker.color.r = 0.0;
    wpt_marker.color.g = 0.0;
    wpt_marker.color.b = 1.0;
    wpt_marker.color.a = 1.0;
    wpt_marker_pub_->publish(wpt_marker);


    /*
        Tracking Waypoint Visualization
    */
    visualization_msgs::msg::Marker track_wpt_marker;
    track_wpt_marker.header.frame_id = map_frame_;
    track_wpt_marker.header.stamp = this->get_clock()->now();
    track_wpt_marker.id = 1;      // This needs to be a unique number per marker
    track_wpt_marker.type = visualization_msgs::msg::Marker::SPHERE;
    
    tf2::Vector3 trk_wpt(tracking_waypoint_.x, tracking_waypoint_.y, 0.0);
    trk_wpt = transform_ego_T_map_.inverse() * trk_wpt;

    track_wpt_marker.pose.position.x = trk_wpt.getX();
    track_wpt_marker.pose.position.y = trk_wpt.getY();

    track_wpt_marker.scale.x = 0.2;
    track_wpt_marker.scale.y = 0.2;
    track_wpt_marker.scale.z = 0.2;

    // Color values must be [0, 1]
    track_wpt_marker.color.r = 1.0;
    track_wpt_marker.color.g = 0.0;
    track_wpt_marker.color.b = 1.0;
    track_wpt_marker.color.a = 1.0;
    track_wpt_marker_pub_->publish(track_wpt_marker);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
