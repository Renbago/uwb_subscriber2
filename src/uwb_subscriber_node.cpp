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
std::vector<double> distance_list;
std::vector<double> lastDistance1;
std::vector<double> lastDistance2;
std::vector<double> lastDistance3;
std::vector<double> rx_power_list;
std::vector<double> robot_x;
std::vector<double> robot_y;
std::vector<double> object_x;
std::vector<double> object_y;

double robotPositionX_;
double robotPositionY_;
double robotRotationZ_;
double robotRotationW_;
double objecPositionX_;
double objecPositionY_;
double robotRotationYaw_ = 0.0;
double polygon_latitude_ = 0.0;
double polygon_longitude_ = 0.05;
double robot_yaw;

std::vector<std::chrono::steady_clock::time_point> last_occurrence_list;
std::vector<std::vector<double>> recentMeasurements; // A new vector to store recent measurements for each anchor

struct Point {
    double x, y;
};

// Path tracking parameters
double first_distance = 0.0;
double path_distance_ = 0.0;

// Gradient Descent Parameters
Point estimatedPos = {0.0, 0.0}; // Initial guess
double learningRate = 0.04;
int maxIterations = 1000;
double tolerance = 1e-5;

// Standard deviation filter parameters
const size_t N = 10; // Example: consider the last 10 measurements
const double M = 1.0; // Example: filter measurements within 2 standard deviations of the mean
const int NUM_POINTS = 3;

//UWB data parameters
std::string uwbData;
std::string id;
double distance_;
double rxPower;

bool calculatable = true;
bool first = true;

double calculateDistance(const Point& p1, const Point& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

Point trilateration(const Point& A, double d1, const Point& B, double d2, const Point& C, double d3) {
    // Using the trilateration formulas to calculate x and y
    double a = 2*B.x - 2*A.x;
    double b = 2*B.y - 2*A.y;
    double c = d1*d1 - d2*d2 - A.x*A.x + B.x*B.x - A.y*A.y + B.y*B.y;
    double d = 2*C.x - 2*B.x;
    double e = 2*C.y - 2*B.y;
    double f = d2*d2 - d3*d3 - B.x*B.x + C.x*C.x - B.y*B.y + C.y*C.y;

    double x = (c*e - f*b) / (e*a - b*d);
    double y = (c*d - a*f) / (b*d - a*e);

    return Point{x, y};
}

// Function to calculate mean
double calculateMean(const std::vector<double>& values) {
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return sum / values.size();
}

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
    for (double measurement : measurements) 
    {
        if (std::abs(measurement - mean) <= nStdDev * stdDev) 
        {
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
    void yawCalculation(double robot_z, double robot_w)
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
        objecPositionX_ = newX1;
        objecPositionY_ = newY1;
    }

    void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        robotPositionX_ = msg->pose.position.x;
        robotPositionY_ = msg->pose.position.y;
        robotRotationZ_ = msg->pose.orientation.z;
        robotRotationW_ = msg->pose.orientation.w;
        yawCalculation(robotRotationZ_, robotRotationW_);
        rotateCoordinates(robotPositionX_, robotPositionY_, robot_yaw);
        Point object = {objecPositionX_, objecPositionY_};
        if (first)
        {
            first_distance = distance_;
            first = false;
        }
        else
        {
            path_distance_ = std::abs(first_distance - distance_);
        }
        if (path_distance_ >= 0.25 && calculatable)
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
                distance_list.push_back(distance_); // This initial distance could also be placed in a temporary buffer if needed
                lastDistance1.push_back(distance_);
                lastDistance2.push_back(0);
                lastDistance3.push_back(0);
                rx_power_list.push_back(rxPower);
                last_occurrence_list.push_back(std::chrono::steady_clock::now());
                recentMeasurements.push_back(std::vector<double>{distance_}); // Initialize the recent measurements buffer for this anchor
                index = recentMeasurements.size() - 1; // Set index to the last added element
            }
            else
            {   
                // Existing anchor, find its index
                index = std::distance(anchor_addr_list.begin(), it);
                recentMeasurements[index].push_back(distance_); // Add new measurement to buffer
                
                if (recentMeasurements[index].size() > N) // Assuming N is defined elsewhere
                {
                    // Apply standard deviation filter
                    double mean = calculateMean(recentMeasurements[index]);
                    double stdDev = calculateStdDev(recentMeasurements[index], mean);
        
                    // Optionally filter out measurements
                    recentMeasurements[index].erase(
                        std::remove_if(recentMeasurements[index].begin(), recentMeasurements[index].end(),
                                    [mean, stdDev](double x) { return std::abs(x - mean) > M * stdDev; }),
                        recentMeasurements[index].end());
        
                    // Use the mean of the filtered measurements as the distance
                    std::vector<double> filteredMeasurements = filterMeasurements(recentMeasurements[index], 2); // Filtering with 2 standard deviations
                    // distance_ = calculateMean(filteredMeasurements);
                }
        
                // Update distance and power lists
                distance_list[index] = distance_;
                rx_power_list[index] = rxPower;
                last_occurrence_list[index] = std::chrono::steady_clock::now();
        
                // Update last distances for trilateration
                lastDistance3[index] = lastDistance2[index];
                lastDistance2[index] = lastDistance1[index];
                lastDistance1[index] = distance_;
            }

            if (object_x.size() > 3)
            {           

                std::cout << "Last 3 Pos: (" << object_x[0] << ", " << object_y[0] << "), (" << object_x[1] << ", " 
                        << object_y[1] << "), (" << object_x[2] << ", " << object_y[2] << ")" << std::endl;
                
                std::cout << "Last 3 Distance: (" << lastDistance1[0] << ", " << lastDistance2[0] << ", " << lastDistance3[0] << ")" << std::endl;
                
                if (0 == (std::any_of(lastDistance1.begin(), lastDistance1.end(), [](double distance) { return distance == 0; }) ||
                    std::any_of(lastDistance2.begin(), lastDistance2.end(), [](double distance) { return distance == 0; }) ||
                    std::any_of(lastDistance3.begin(), lastDistance3.end(), [](double distance) { return distance == 0; })))
                {
                    std::vector<Point> anchors = {{object_x[0], object_y[0]}, {object_x[1], object_y[1]}, {object_x[2], object_y[2]}};
                    std::vector<double> distances = {lastDistance1[0], lastDistance2[0], lastDistance3[0]};    
                    Point estimatedPos = trilateration(anchors[0], distances[0], anchors[1], distances[1], anchors[2], distances[2]);
                    std::cout << "Estimated Position: (" << estimatedPos.x << ", " << estimatedPos.y << ")" << std::endl;
                }
                object_x.erase(object_x.begin());
                object_y.erase(object_y.begin());
                robot_x.erase(robot_x.begin());
                robot_y.erase(robot_y.begin());
            }
            first_distance = distance_;
            path_distance_ = 0.0;
        }
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
