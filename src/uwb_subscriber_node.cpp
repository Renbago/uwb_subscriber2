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
std::vector<std::vector<float>> recentMeasurements; // A new vector to store recent measurements for each anchor

// Function to calculate mean
double calculateMean(const std::vector<double>& values) {
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return sum / values.size();
}

// Function to calculate standard deviation
double calculateStdDev(const std::vector<double>& values, double mean) {
    double squareSum = std::accumulate(values.begin(), values.end(), 0.0, 
                                       [mean](double acc, double val) { return acc + (val - mean) * (val - mean); });
    return std::sqrt(squareSum / values.size());
}

// Function to filter measurements based on standard deviation
std::vector<double> filterMeasurements(const std::vector<double>& measurements, double nStdDev) {
    double mean = calculateMean(measurements);
    double stdDev = calculateStdDev(measurements, mean);
    
    std::vector<double> filtered;
    for (double measurement : measurements) {
        if (std::abs(measurement - mean) <= nStdDev * stdDev) {
            filtered.push_back(measurement);
        }
    }
    return filtered;
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
        return robotYaw;
    }
 
   
    void rotateCoordinates(double robotPositionX, double robotPositionY, double robotRotationYaw) 
    {
        // Set the X and Y coordinates
        vec estimatedRobot_x_coordinates = {0.05};
        vec estimatedRobot_y_coordinates = {0.0};
    
        // Create a 2xN matrix
        mat points = join_vert(estimatedRobot_x_coordinates.t(), estimatedRobot_y_coordinates.t());
    
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
        // cout << "Rotated X Coordinates: " << newX1 << endl;
        // cout << "Rotated Y Coordinates: " << newY1 << endl;
        objecPositionX_ = newX1;
        objecPositionY_ = newY1;
    }

    

    void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        robotPositionX_ = msg->pose.position.x;
        robotPositionY_ = msg->pose.position.y;
        robotRotationZ_ = msg->pose.orientation.z;
        robotRotationW_ = msg->pose.orientation.w;
        robotRotationYaw_ = baha_calculation(robotRotationZ_, robotRotationW_);
        rotateCoordinates(robotPositionX_, robotPositionY_,  robotRotationYaw_);
        object_x.push_back(objecPositionX_);
        object_y.push_back(objecPositionY_);   
        
        if (object_x.size() > 3)
        {
            // RCLCPP_INFO(this->get_logger(), "Last 3 Robot Pose: (%f, %f), (%f, %f), (%f, %f)",
            // object_x[0], object_y[0], object_x[1], object_y[1], object_x[2], object_y[2]);
            if (calculate == 1 && 0 == (std::any_of(lastDistance1.begin(), lastDistance1.end(), [](float distance) { return distance == 0; }) ||
                std::any_of(lastDistance2.begin(), lastDistance2.end(), [](float distance) { return distance == 0; }) ||
                std::any_of(lastDistance3.begin(), lastDistance3.end(), [](float distance) { return distance == 0; })))
            {
                for (size_t i = 0; i < anchor_addr_list.size(); i++)
                {
                    // RCLCPP_INFO(this->get_logger(), "Anchor %s - Distance: %f, RX Power: %f",
                                // anchor_addr_list[i].c_str(), distance_list[i], rx_power_list[i]);
                    std::cout << "robot pos" << msg->pose.position.x << " " << msg->pose.position.y << std::endl;
                    std::cout << "object pos:" << object_x[0] << " " << object_y[0] << std::endl;
                    std::cout << "robot angle " << robotRotationYaw_ << std::endl;

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

                    // std::cout << "Estimated position: (" << p.x << ", " << p.y << ")" << std::endl;
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
    
        // First, check if the anchor is already known
        auto it = std::find(anchor_addr_list.begin(), anchor_addr_list.end(), id);
        size_t index;
        if (it == anchor_addr_list.end())   
        {
            // New anchor, add its details to the lists
            anchor_addr_list.push_back(id);
            distance_list.push_back(distance); // This initial distance could also be placed in a temporary buffer if needed
            lastDistance1.push_back(distance);
            lastDistance2.push_back(0);
            lastDistance3.push_back(0);
            rx_power_list.push_back(rxPower);
            last_occurrence_list.push_back(std::chrono::steady_clock::now());
            recentMeasurements.push_back(std::vector<float>{distance}); // Initialize the recent measurements buffer for this anchor
            index = recentMeasurements.size() - 1; // Set index to the last added element
        }
        else
        {
            // Existing anchor, find its index
            index = std::distance(anchor_addr_list.begin(), it);
            recentMeasurements[index].push_back(distance); // Add new measurement to buffer
    
            if (recentMeasurements[index].size() > N) // Assuming N is defined elsewhere
            {
                // Apply standard deviation filter
                float mean = calculateMean(recentMeasurements[index]);
                float stdDev = calculateStdDev(recentMeasurements[index], mean);
    
                // Optionally filter out measurements
                recentMeasurements[index].erase(
                    std::remove_if(recentMeasurements[index].begin(), recentMeasurements[index].end(),
                                   [mean, stdDev](float x) { return std::abs(x - mean) > M * stdDev; }),
                    recentMeasurements[index].end());
    
                // Use the mean of the filtered measurements as the distance
                distance = calculateMean(recentMeasurements[index]);
            }
    
            // Update distance and power lists
            distance_list[index] = distance;
            rx_power_list[index] = rxPower;
            last_occurrence_list[index] = std::chrono::steady_clock::now();
    
            // Update last distances for trilateration
            lastDistance3[index] = lastDistance2[index];
            lastDistance2[index] = lastDistance1[index];
            lastDistance1[index] = distance;
            calculate = true;
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
