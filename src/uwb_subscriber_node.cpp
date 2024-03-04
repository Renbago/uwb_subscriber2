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

std::vector<std::string> anchor_addr_list;
std::vector<float> distance_list;
std::vector<float> lastDistance1;
std::vector<float> lastDistance2;
std::vector<float> lastDistance3;
std::vector<float> rx_power_list;
std::vector<float> object_x;
std::vector<float> object_y;

float robotPositionX_;
float robotPositionY_;
float robotRotationZ_;
float robotRotationW_;
float objecPositionX_;
float objecPositionY_;
float robotRotationYaw_ = 0.0;
float polygon_latitude_ = 0.0;
float polygon_longitude_ = 0.05;

std::vector<std::chrono::steady_clock::time_point> last_occurrence_list;


const int NUM_POINTS = 3;
bool calculate = false;

// Structure to represent a point in 2D space
struct Point {
    double x, y;
};

// Function to calculate the sum of squared differences
double F(const Point& p, const Point points[], const double distances[]) {
    double sum = 0.0;
    for (int i = 0; i < NUM_POINTS; ++i) {
        double dx = p.x - points[i].x;
        double dy = p.y - points[i].y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double diff = distance - distances[i];
        sum += diff * diff;
    }
    return sum;
}

// Function to calculate the gradient of F
Point grad_F(const Point& p, const Point points[], const double distances[]) {
    Point grad = {0.0, 0.0};
    for (int i = 0; i < NUM_POINTS; ++i) {
        double dx = p.x - points[i].x;
        double dy = p.y - points[i].y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance == 0) continue; // Prevent division by zero
        double diff = distance - distances[i];
        grad.x += (dx / distance) * diff;
        grad.y += (dy / distance) * diff;
    }
    grad.x *= 2;
    grad.y *= 2;
    return grad;
}

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
    double baha_calculation(float robot_z, float robot_w)
    {
        double robotRotationZ = robot_z;
        double robotRotationW = robot_w;
        double robotRotationYaw = (atan2(2.0*(robotRotationW*robotRotationZ), 1.0 - 2.0*(robotRotationZ*robotRotationZ)))*180.0/M_PI ;
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
        return robotYaw;
    }
 
    void rotate_coordinates(double robotPositionX, double robotPositionY, double polygon_latitude, double polygon_longitude, double robotRotationYaw)
    {
        
        // resion of yhe 0.5  in front of the robot polygon start point 
        // this calculation based for rotated 90 degrees referance coordinates system (a.k.a map frame)
        arma::vec estimatedRobot_y_coordinates = {polygon_longitude};
        arma::vec estimatedRobot_x_coordinates = {polygon_latitude};

        arma::mat points(2, estimatedRobot_x_coordinates.size());
        points.row(0) = estimatedRobot_x_coordinates.t(); 
        points.row(1) = estimatedRobot_y_coordinates.t();

        double radians = arma::datum::pi / 180.0 * robotRotationYaw;
        arma::mat22 rotationMatrix;
        rotationMatrix << std::cos(radians) << std::sin(radians) << arma::endr
                       << -std::sin(radians) << std::cos(radians) << arma::endr;

        arma::mat rotatedPoints = rotationMatrix * points;

        // Update the global coordinates of the polygon
        // Assuming you want the first element of the resulting operation
        objecPositionX_ = (rotatedPoints.row(1) + robotPositionX).eval()(0,0);
        objecPositionY_ = (rotatedPoints.row(0) + robotPositionY).eval()(0,0);

        printf("Object Position: (%f, %f)\n", objecPositionX_, objecPositionY_);
              
    }
    

    void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        robotPositionX_ = msg->pose.position.x;
        robotPositionY_ = msg->pose.position.y;
        robotRotationZ_ = msg->pose.orientation.z;
        robotRotationW_ = msg->pose.orientation.w;
        robotRotationYaw_ = baha_calculation(robotRotationZ_, robotRotationW_);
        rotate_coordinates(robotPositionX_, robotPositionY_,  polygon_latitude_, polygon_longitude_, robotRotationYaw_);
        object_x.push_back(objecPositionX_);
        object_y.push_back(objecPositionY_);   
        
        if (object_x.size() > 3)
        {
            RCLCPP_INFO(this->get_logger(), "Last 3 Robot Pose: (%f, %f), (%f, %f), (%f, %f)",
            object_x[0], object_y[0], object_x[1], object_y[1], object_x[2], object_y[2]);
            if (calculate == 1 && 0 == (std::any_of(lastDistance1.begin(), lastDistance1.end(), [](float distance) { return distance == 0; }) ||
                std::any_of(lastDistance2.begin(), lastDistance2.end(), [](float distance) { return distance == 0; }) ||
                std::any_of(lastDistance3.begin(), lastDistance3.end(), [](float distance) { return distance == 0; })))
            {
                for (size_t i = 0; i < anchor_addr_list.size(); i++)
                {
                    RCLCPP_INFO(this->get_logger(), "Anchor %s - Distance: %f, RX Power: %f",
                                anchor_addr_list[i].c_str(), distance_list[i], rx_power_list[i]);

                    Point points[NUM_POINTS] = {{object_x[0], object_y[0]}, {object_x[1],object_y[1]}, {object_x[2],object_y[2]}};
                    double distances[NUM_POINTS] = {lastDistance1[i], lastDistance2[i], lastDistance3[i]};
                    
                    // Adjusted initial guess for the unknown point's position closer to the last known robot pose
                    Point p = {-2.727810, 8.637439};

                    // Gradient descent parameters
                    double learning_rate = 0.01;
                    int iterations = 1000;
                    double tolerance = 1e-6; // Increased tolerance due to potential error in distance measurements

                    // Gradient Descent Algorithm
                    for (int i = 0; i < iterations; ++i) {
                        Point grad = grad_F(p, points, distances);
                        Point new_p = {p.x - learning_rate * grad.x, p.y - learning_rate * grad.y};

                        // Check for convergence
                        double change = std::sqrt((new_p.x - p.x) * (new_p.x - p.x) + (new_p.y - p.y) * (new_p.y - p.y));
                        if (change < tolerance) {
                            std::cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO";
                            std::cout << "Converged after " << i << " iterations." << std::endl;
                            break;
                        }

                        p = new_p;
                    }

                    std::cout << "Estimated position: (" << p.x << ", " << p.y << ")" << std::endl;
                }
            }
            object_x.erase(object_x.begin());
            object_y.erase(object_y.begin());
        }
    }

    void uwb_data_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string uwbData = msg->data;
        std::string id = uwbData.substr(0, 4);
        float distance = std::stof(uwbData.substr(4, 5)) / 1000;
        float rxPower = std::stof(uwbData.substr(9)) / 1000 * -1;

        auto it = std::find(anchor_addr_list.begin(), anchor_addr_list.end(), id);
        if (it == anchor_addr_list.end())   
        {
            anchor_addr_list.push_back(id);
            distance_list.push_back(distance);
            lastDistance1.push_back(distance);
            lastDistance2.push_back(0);
            lastDistance3.push_back(0);
            rx_power_list.push_back(rxPower);
            last_occurrence_list.push_back(std::chrono::steady_clock::now());
        }
        else 
        {    
            auto index = it - anchor_addr_list.begin();

            if (distance_list[index] - distance > 0.15f || distance_list[index] - distance < 0.15f)
            {
                distance_list[index] = distance;
                lastDistance3[index] = lastDistance2[index];
                lastDistance2[index] = lastDistance1[index];
                lastDistance1[index] = distance;
                calculate = true;
            }
            else
            {
                calculate = false;
            }
            rx_power_list[index] = rxPower;
            last_occurrence_list[index] = std::chrono::steady_clock::now();
        }
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
                distance_list.erase(distance_list.begin() + i);
                lastDistance1.erase(lastDistance1.begin() + i);
                lastDistance2.erase(lastDistance2.begin() + i);
                lastDistance3.erase(lastDistance3.begin() + i);
                rx_power_list.erase(rx_power_list.begin() + i);
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
