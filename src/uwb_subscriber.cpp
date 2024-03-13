#include "uwb_subscriber.h"

using namespace std::chrono_literals;
using namespace uwb_subscriber;
using namespace arma;
using namespace std;

uwbSubscriber::uwbSubscriber(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
  
    initialize();
}

uwbSubscriber::~uwbSubscriber(){
}

void uwbSubscriber::updateParamsUtil(){

    RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), " updateParamsUtil() called");

    nh_->declare_parameter("loop_rate", rclcpp::PARAMETER_DOUBLE);
    nh_->get_parameter_or("loop_rate", p_loop_rate_, 10.0);

    p_sampling_time_ = 1.0 / p_loop_rate_;
    timer_ = nh_->create_wall_timer(p_sampling_time_ * 1s, std::bind(&uwbSubscriber::timerCallback, this));

    robot_pose_subscriber_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "robot_pose_stamped", 10, std::bind(&uwbSubscriber::robotPoseCallback, this, std::placeholders::_1));
    uwb_data_subscriber_ = nh_->create_subscription<std_msgs::msg::String>(
        "uwb_data", 10, std::bind(&uwbSubscriber::uwbDataCallback, this, std::placeholders::_1));

    distance_vis_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("distance_uwb", 0);
}

void uwbSubscriber::timerCallback(){
    RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), "Timer callback called");
}

void uwbSubscriber::robotRotationCallback(){

    double robotRotationYaw = std::atan2(2.0 * (robot_orientation_.w * robot_orientation_.z), 
                                                1.0 - 2.0 * (robot_orientation_.z * robot_orientation_.z)) * 180.0 / M_PI + 90;

    // Adjusting yaw to be within the range [-180, 180]
    if (robotRotationYaw > 180) {
        robotRotationYaw -= 360;
    } else if (robotRotationYaw < -180) {
        robotRotationYaw += 360;
    }
    robotRotationYaw_ = robotRotationYaw;

}

arma::mat uwbSubscriber::rotateCoordinates() {
    // resion of yhe 0.5  in front of the robot polygon start point 
    // this calculation based for rotated 90 degrees referance coordinates system (a.k.a map frame)
    arma::vec estimatedRobot_x_coordinates = {-0.3};
    arma::vec estimatedRobot_y_coordinates = {0.0};

    arma::mat points(2, estimatedRobot_x_coordinates.size());
    points.row(0) = estimatedRobot_x_coordinates.t(); 
    points.row(1) = estimatedRobot_y_coordinates.t();

    double radians = arma::datum::pi / 180.0 * robotRotationYaw_;
    arma::mat22 rotationMatrix;
    rotationMatrix << std::cos(radians) << std::sin(radians) << arma::endr
                   << -std::sin(radians) << std::cos(radians) << arma::endr;
    
    arma::mat rotatedPoints = rotationMatrix * points;

    // Update the global coordinates of the polygon
    anchor_global_coordinates_.x = rotatedPoints(1) + robot_position_.x;
    anchor_global_coordinates_.y = rotatedPoints(0) + robot_position_.y;

    return rotatedPoints;
} 

void uwbSubscriber::uwbDistanceCalculation(){
    RCLCPP_INFO_STREAM(nh_->get_logger(), "UWB distance calculation called");
    
    robot_path_diff += std::abs(uwb_infos_.last_distance - uwb_infos_.distance);

    if (robot_path_diff >= 0.4){
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Robot path difference: " << robot_path_diff);
        
        object_infos_.robot_x.push_back(robot_position_.x);
        object_infos_.robot_y.push_back(robot_position_.y);
        object_infos_.object_x.push_back(anchor_global_coordinates_.x);
        object_infos_.object_y.push_back(anchor_global_coordinates_.y);

        auto it = std::find(anchor_infos_.anchor_addr_list.begin(), anchor_infos_.anchor_addr_list.end(), uwb_infos_.id);
        size_t index;
        if (it == anchor_infos_.anchor_addr_list.end()){
            anchor_infos_.anchor_addr_list.push_back(uwb_infos_.id);
            anchor_infos_.distances.push_back(anchor_infos_.anchor_distance);
            anchor_infos_.anchor_distance.push_back(uwb_infos_.distance);
            anchor_infos_.last_occurrence_list.push_back(std::chrono::steady_clock::now());
            anchor_infos_.recentMeasurements.push_back(std::vector<double>{uwb_infos_.distance});
            index = anchor_infos_.recentMeasurements.size() - 1;
        }

        else{
            RCLCPP_INFO_STREAM(nh_->get_logger(), "Anchor already known");
            index = std::distance(anchor_infos_.anchor_addr_list.begin(), it);
            anchor_infos_.anchor_distance.push_back(uwb_infos_.distance);
            if (anchor_infos_.anchor_distance.size() > 10){
                anchor_infos_.anchor_distance.erase(anchor_infos_.anchor_distance.begin());
            }
            anchor_infos_.distances[index] = anchor_infos_.anchor_distance;
            anchor_infos_.last_occurrence_list[index] = std::chrono::steady_clock::now(); 
        }
    
        RCLCPP_INFO(nh_->get_logger(), "objec_x size: %ld", object_infos_.object_x.size());
        RCLCPP_INFO(nh_->get_logger(), "distances[0] siez : %ld", anchor_infos_.distances[0].size());
        if (object_infos_.object_x.size() >= 10){

            RCLCPP_INFO(nh_->get_logger(), "Last 10 Robot Pos:");
            for (int i = 0; i < object_infos_.object_x.size(); i++){
                RCLCPP_INFO(nh_->get_logger(), "Robot Pos: %f, %f", object_infos_.robot_x[i], object_infos_.robot_y[i]);
            }

            RCLCPP_INFO(nh_->get_logger(), "Last 10 Object Pos:");
            for (int i = 0; i < object_infos_.object_x.size(); i++)
            {
                RCLCPP_INFO(nh_->get_logger(), "Object Pos: %f, %f", object_infos_.object_x[i], object_infos_.object_y[i]);
            }

            RCLCPP_INFO(nh_->get_logger(), "Last 10 Anchor Distances:");
            for (int i = 0; i < anchor_infos_.distances[0].size(); i++)
            {
                RCLCPP_INFO(nh_->get_logger(), "Anchor Distances: %f", anchor_infos_.distances[0][i]);
            }

            object_infos_.object_x.erase(object_infos_.object_x.begin());
            object_infos_.object_y.erase(object_infos_.object_y.begin());
            object_infos_.robot_x.erase(object_infos_.robot_x.begin());
            object_infos_.robot_y.erase(object_infos_.robot_y.begin());

        }
        robot_path_diff = 0;
    }
    uwb_infos_.last_distance = uwb_infos_.distance;
    RCLCPP_INFO(nh_->get_logger(), "Last distance: %f", uwb_infos_.last_distance);

} 

void uwbSubscriber::publishCircleMarker() {
    auto circle = visualization_msgs::msg::Marker();
    auto start_pose_circle = visualization_msgs::msg::Marker();

    start_pose_circle.header.frame_id = "map"; 
    start_pose_circle.header.stamp = nh_->get_clock()->now();
    start_pose_circle.ns = "start_circle";
    start_pose_circle.id = 1;
    start_pose_circle.type = visualization_msgs::msg::Marker::CYLINDER;
    start_pose_circle.action = visualization_msgs::msg::Marker::ADD;
    start_pose_circle.pose.orientation.w = 1.0;
    start_pose_circle.scale.x = 0.2;
    start_pose_circle.scale.y = 0.2;
    start_pose_circle.scale.z = 0.01;
    start_pose_circle.color.r = 0.0;
    start_pose_circle.color.g = 0.0;
    start_pose_circle.color.b = 0.0;
    start_pose_circle.color.a = 0.8;
    start_pose_circle.pose.position.x = -0.765395322481599;
    start_pose_circle.pose.position.y = -3.3472860311432036;
    start_pose_circle.pose.position.z = 0.1;
    start_pose_circle.pose.orientation.x = 0.0;
    start_pose_circle.pose.orientation.y = 0.0;
    start_pose_circle.pose.orientation.z = 0.0;
    start_pose_circle.pose.orientation.w = 1.0;

    circle.header.frame_id = "map"; 
    circle.header.stamp = nh_->get_clock()->now();
    circle.ns = "uwb_circle";
    circle.id = 0;
    circle.type = visualization_msgs::msg::Marker::CYLINDER;
    circle.action = visualization_msgs::msg::Marker::ADD;
    circle.pose.orientation.w = 1.0;
    circle.scale.x = uwb_infos_.last_distance * 2;
    circle.scale.y = uwb_infos_.last_distance * 2;
    circle.scale.z = 0.01;
    circle.color.g = 1.0;
    circle.color.a = 0.3;
    circle.pose.position.x = robot_position_.x;
    circle.pose.position.y = robot_position_.y;
    circle.pose.position.z = 0.1;
    circle.pose.orientation.x = 0.0;
    circle.pose.orientation.y = 0.0;
    circle.pose.orientation.z = 0.0;
    circle.pose.orientation.w = 1.0;

    distance_vis_pub_->publish(circle);
    distance_vis_pub_->publish(start_pose_circle);

}


void uwbSubscriber::robotPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg){
    robot_position_ = msg->pose.position;
    robot_orientation_ = msg->pose.orientation;
    RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), "Robot position received: ");
    robotRotationCallback();
    arma::mat rotatedPoints = rotateCoordinates();
    uwbDistanceCalculation();
    publishCircleMarker();
}

void uwbSubscriber::uwbDataCallback(const std_msgs::msg::String::ConstSharedPtr& msg){
    uwb_msg_ = *msg;
    uwb_infos_.id = uwb_msg_.data.substr(0, 4);
    uwb_infos_.distance = std::stod(uwb_msg_.data.substr(4, 5)) / 1000;
    uwb_infos_.rxPower = std::stod(uwb_msg_.data.substr(9)) / 1000 * -1;
    RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), "UWB data received: ");

}



    