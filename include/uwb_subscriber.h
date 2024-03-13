#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath> 
#include <armadillo>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace uwb_subscriber 
{

class uwbSubscriber 
{
    public:
        uwbSubscriber(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_local);
        ~uwbSubscriber();

    private:

    void updateParamsUtil();
    void timerCallback();
    void robotPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void uwbDataCallback(const std_msgs::msg::String::ConstSharedPtr& msg);
    void initialize() { std_srvs::srv::Empty empt; updateParamsUtil(); }

    void robotRotationCallback();
    arma::mat rotateCoordinates();
    void uwbDistanceCalculation();
    void publishCircleMarker();

    typedef struct
    {
        std::string id;
        double distance;
        double rxPower;
        double last_distance{0.0};
        
    } uwbStruct;
    uwbStruct uwb_infos_;
    
    typedef struct 
    {
        std::vector<std::vector<double>> distances;
        std::vector<std::string> anchor_addr_list;
        std::vector<double> anchor_distance;
        std::vector<std::chrono::steady_clock::time_point> last_occurrence_list;
        std::vector<std::vector<double>> recentMeasurements;
    } anchorStruct;
    anchorStruct anchor_infos_;
    
    typedef struct
    {
        std::vector<double> robot_x;
        std::vector<double> robot_y;
        std::vector<double> object_x;
        std::vector<double> object_y;
    } objectStruct;
    objectStruct object_infos_; 

    typedef struct
    {
        double x, y;
    } anchorGlobalCoordinates;
    anchorGlobalCoordinates anchor_global_coordinates_;

    geometry_msgs::msg::Point robot_position_;
    geometry_msgs::msg::Quaternion robot_orientation_;
    std_msgs::msg::String uwb_msg_;

    std::shared_ptr<rclcpp::Node> nh_;
    std::shared_ptr<rclcpp::Node> nh_local_;
    std::vector<std::string> anchor_addr_list_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uwb_data_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr distance_vis_pub_;

    arma::vec estimatedRobot_x_coordinates_;
    arma::vec estimatedRobot_y_coordinates_;
    arma::mat rotated_polygon_points_;

    bool first_{true};
    double p_loop_rate_;
    double p_sampling_time_;
    double robotRotationYaw_;
    double robot_path_diff{0.0};
};

}