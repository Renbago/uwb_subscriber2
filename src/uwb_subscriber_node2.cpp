#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <chrono>
#include <iostream>
#include <cmath>
#include <array>
#include <armadillo>

using namespace arma;
using namespace std;

std::vector<std::string> anchor_addr_list;
std::vector<double> robot_x;
std::vector<double> robot_y;
std::vector<double> object_x;
std::vector<double> object_y;
std::vector<double> anchor_distance;
std::vector<std::vector<double>> distances;
std::vector<std::vector<double>> anchors;

double robotPositionX_;
double robotPositionY_;
double robotRotationZ_;
double robotRotationW_;
double objecPositionX_;
double objecPositionY_;
double robotRotationYaw_ = 0.0;
double robot_yaw;

std::vector<std::chrono::steady_clock::time_point> last_occurrence_list;
std::vector<std::vector<double>> recentMeasurements; // A new vector to store recent measurements for each anchor

struct Point {
    double x, y;
};

// Path tracking parameters
double last_distance = 0.0;
double path_distance_ = 0.0;

// Standard deviation filter parameters
const size_t N = 10; // Example: consider the last 10 measurements
const double M = 1.0; // Example: filter measurements within 2 standard deviations of the mean
const int NUM_POINTS = 3;

//UWB data parameters
std::string uwbData;
std::string id;
double distance_;
double rxPower;

bool first = true;

class UWBSubscriberNode : public rclcpp::Node
{
public:
    UWBSubscriberNode() : Node("uwb_subscriber_node")
    {
        robot_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/SR2T1/robot_pose_stamped", 10, std::bind(&UWBSubscriberNode::robot_pose_callback, this, std::placeholders::_1));
        uwb_data_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/SR2T1/uwb_data", 10, std::bind(&UWBSubscriberNode::uwb_data_callback, this, std::placeholders::_1));

        // Create a timer to check for inactive IDs every 30 seconds
        inactive_ids_timer_ = this->create_wall_timer(std::chrono::seconds(30),
            std::bind(&UWBSubscriberNode::check_inactive_ids, this));
    }

private:
    void yawCalculation(double orientation_z, double orientation_w)
    {
        double robotRotationZ = robot_z;
        double robotRotationW = robot_w;
        double robotRotationYaw = (atan2(2.0*(robotRotationW*robotRotationZ), 1.0 - 2.0*(robotRotationZ*robotRotationZ)))*180.0/M_PI + 90;
        double robotYaw;
        if(robotRotationYaw >180)
        {
          robotYaw = robotRotationYaw - 360;
        }
        else if(robotRotationYaw < -180){
          robotYaw = robotRotationYaw + 360;
        }
        else{
          robotYaw = robotRotationYaw;
        }
        robot_yaw = robotYaw;   
    }

    void rotateCoordinates(double robotPositionX, double robotPositionY, double robotRotationYaw) 
    {
        // Set the X and Y coordinates
        vec estimatedRobot_x_coordinates = {0.05};
        vec estimatedRobot_y_coordinates = {0.0};

        // Create a 2xN matrix
        arma::mat points = join_vert(estimatedRobot_x_coordinates.t(), estimatedRobot_y_coordinates.t());

        // Create rotation matrix
        double radians = datum::pi * robotRotationYaw / 180.0; // Convert degrees to radians
        mat rotationMatrix = {
            {cos(radians), sin(radians)},
            {-sin(radians), cos(radians)}
        };
        // Rotate points
        mat rotatedPoints = rotationMatrix * points;

        // Use the coordinates after rotation
        double newX1 = rotatedPoints(1, 0) + robotPositionX;
        double newY1 = rotatedPoints(0, 0) + robotPositionY;
        // For other points, you can similarly use newX and newY values
        objecPositionX_ = newX1;
        objecPositionY_ = newY1;
    }



    void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        robotPositionX_ = msg->pose.position.x;
        robotPositionY_ = msg->pose.position.y;
        robotRotationZ_ = msg->pose.orientation.z;
        robotRotationW_ = msg->pose.orientation.w;
        yawCalculation(msg->pose.orientation.z, msg->pose.orientation.w);
        rotateCoordinates(msg->pose.position.x, msg->pose.position.y, robot_yaw);

        if (first)
        {
            last_distance = distance_;
            first = false;
        }
        else
        {
            path_distance_ += std::abs(last_distance - distance_);
        }

        if (path_distance_ >= 0.6)
        {
            rotateCoordinates(robotPositionX_, robotPositionY_,  robotRotationYaw_);
            
            robot_x.push_back(robotPositionX_);
            robot_y.push_back(robotPositionY_);
            object_x.push_back(objecPositionX_);
            object_y.push_back(objecPositionY_);

            
            // First, check if the anchor is already known
            auto it = std::find(anchor_addr_list.begin(), anchor_addr_list.end(), id);
            
            size_t index;
            if (it == anchor_addr_list.end())
            {
                // New anchor, add its details to the lists
                anchor_addr_list.push_back(id);
                distances.push_back(anchor_distance);
                anchor_distance.push_back(distance_);
                last_occurrence_list.push_back(std::chrono::steady_clock::now());
                recentMeasurements.push_back(std::vector<double>{distance_}); // Initialize the recent measurements buffer for this anchor
                index = recentMeasurements.size() - 1; // Set index to the last added element
            }
            else
            {   
                std::cout << "Anchor already known" << std::endl;
                // Existing anchor, find its index
                index = std::distance(anchor_addr_list.begin(), it);
                anchor_distance.push_back(distance_);
                if (anchor_distance.size() > 10)
                {
                    anchor_distance.erase(anchor_distance.begin());
                }
                distances[index] = anchor_distance;
                
                // Update distance and power lists
                last_occurrence_list[index] = std::chrono::steady_clock::now();
            }

            RCLCPP_INFO(this->get_logger(), "objec_x size: %d", object_x.size());
            RCLCPP_INFO(this->get_logger(), "distances[0] siez : %d", distances[0].size());
            if (object_x.size() >= 10)
            {
                RCLCPP_INFO(this->get_logger(), "Last 10 Robot Pos:");
                for (int i = 0; i < object_x.size(); i++)
                {
                    RCLCPP_INFO(this->get_logger(), "Robot Pos: %f, %f", robot_x[i], robot_y[i]);
                }

                RCLCPP_INFO(this->get_logger(), "Last 10 Object Pos:");
                for (int i = 0; i < object_x.size(); i++)
                {
                    RCLCPP_INFO(this->get_logger(), "Object Pos: %f, %f", object_x[i], object_y[i]);
                }

                RCLCPP_INFO(this->get_logger(), "Last 10 Anchor Distances:");
                for (int i = 0; i < distances[0].size(); i++)
                {
                    RCLCPP_INFO(this->get_logger(), "Anchor Distances: %f", distances[0][i]);
                }

                object_x.erase(object_x.begin());
                object_y.erase(object_y.begin());
                robot_x.erase(robot_x.begin());
                robot_y.erase(robot_y.begin());
            }
            path_distance_ = 0;
        }
        last_distance = distance_;

    }

    void uwb_data_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        uwbData = msg->data;
        id = uwbData.substr(0, 4);
        distance_ = std::stod(uwbData.substr(4, 5)) / 1000;
        rxPower = std::stod(uwbData.substr(9)) / 1000 * -1;
    }

    void check_inactive_ids()
    {
        auto current_time = std::chrono::steady_clock::now();
        for (size_t i = 0; i < anchor_addr_list.size(); i++)
        {
            auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_occurrence_list[i]);
            if (time_diff.count() >= 30)
            {
                anchor_addr_list.erase(anchor_addr_list.begin() + i);
                last_occurrence_list.erase(last_occurrence_list.begin() + i);
                i--; 
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uwb_data_subscriber_;
    rclcpp::TimerBase::SharedPtr inactive_ids_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UWBSubscriberNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    return 0;
}