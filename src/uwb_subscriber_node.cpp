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
std::vector<std::time_t> lastDistance1_timestamp;
std::vector<double> lastDistance2;
std::vector<std::time_t> lastDistance2_timestamp;
std::vector<double> lastDistance3;
std::vector<std::time_t> lastDistance3_timestamp;
std::vector<double> rx_power_list;
std::vector<double> robot_x;
std::vector<double> robot_y;
std::vector<std::time_t> robot_x_timestamp;
std::vector<std::time_t> robot_y_timestamp;

double robotPositionX_;
double robotPositionY_;
double robotRotationZ_;
double robotRotationW_;
double objecPositionX_;
double objecPositionY_;
double robotRotationYaw_ = 0.0;
double polygon_latitude_ = 0.0;
double polygon_longitude_ = 0.05;

std::vector<std::chrono::steady_clock::time_point> last_occurrence_list;
std::vector<std::vector<double>> recentMeasurements; // A new vector to store recent measurements for each anchor


const size_t N = 10; // Example: consider the last 10 measurements
const double M = 2.0; // Example: filter measurements within 2 standard deviations of the mean
const int NUM_POINTS = 3;

bool calculate = false;

struct Point {
    double x, y;
};


/// Function to calculate the intersection of two circles
std::vector<Point> intersectTwoCircles(Point p1, double r1, Point p2, double r2) {
    std::vector<Point> intersections;

    double d = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));

    // No solution if the circles do not intersect
    if (d > r1 + r2 || d < std::fabs(r1 - r2)) {
        return intersections; // Empty
    }

    double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    double h = std::sqrt(r1 * r1 - a * a);

    Point p0 = {p1.x + a * (p2.x - p1.x) / d, p1.y + a * (p2.y - p1.y) / d};

    // Intersection points
    Point intersect1 = {p0.x + h * (p2.y - p1.y) / d, p0.y - h * (p2.x - p1.x) / d};
    Point intersect2 = {p0.x - h * (p2.y - p1.y) / d, p0.y + h * (p2.x - p1.x) / d};

    intersections.push_back(intersect1);
    if (d != r1 + r2) { // If circles intersect in more than one point
        intersections.push_back(intersect2);
    }

    return intersections;
}

// Basic trilateration function using three points and distances
Point trilateration(const std::vector<Point>& anchors, const std::vector<double>& distances) {
    // Ensure there are exactly three anchors and three distances
    if (anchors.size() != 3 || distances.size() != 3) {
        std::cerr << "Trilateration requires exactly three anchors and three distances" << std::endl;
        return {0.0, 0.0}; // Return a default point in case of error
    }

    // Extracting anchor points
    double x1 = anchors[0].x, y1 = anchors[0].y;
    double x2 = anchors[1].x, y2 = anchors[1].y;
    double x3 = anchors[2].x, y3 = anchors[2].y;

    // Extracting distances
    double d1 = distances[0], d2 = distances[1], d3 = distances[2];

    // Pre-compute the coefficients
    double A = 2*x2 - 2*x1;
    double B = 2*y2 - 2*y1;
    double C = d1*d1 - d2*d2 - x1*x1 - y1*y1 + x2*x2 + y2*y2;
    double D = 2*x3 - 2*x2;
    double E = 2*y3 - 2*y2;
    double F = d2*d2 - d3*d3 - x2*x2 - y2*y2 + x3*x3 + y3*y3;

    // Solve for x and y
    double x = (C*E - F*B) / (E*A - B*D);
    double y = (C*D - A*F) / (B*D - A*E);

    return {x, y};
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
    // double baha_calculation(double robot_z, double robot_w)
    // {
    //     double robotRotationZ = robot_z;
    //     double robotRotationW = robot_w;
    //     double robotRotationYaw = (atan2(2.0*(robotRotationW*robotRotationZ), 1.0 - 2.0*(robotRotationZ*robotRotationZ)))*180.0/M_PI + 90;
    //     double robotYaw;
    //     if(robotRotationYaw >180)
    //     {
    //       robotYaw = robotRotationYaw - 360;
    //     }
    //     else if(robotRotationYaw < -180){
    //       robotYaw = robotRotationYaw + 360;
    //     }
    //     else{
    //       robotYaw = robotRotationYaw;
    //     }
    //     return robotYaw;
    // }
 
   
    // void rotateCoordinates(double robotPositionX, double robotPositionY, double robotRotationYaw) 
    // {
    //     // Set the X and Y coordinates
    //     vec estimatedRobot_x_coordinates = {0.05};
    //     vec estimatedRobot_y_coordinates = {0.0};
    
    //     // Create a 2xN matrix
    //     mat points = join_vert(estimatedRobot_x_coordinates.t(), estimatedRobot_y_coordinates.t());
    
    //     // Create rotation matrix
    //     double radians = datum::pi * robotRotationYaw / 180.0; // Convert degrees to radians
    //     mat rotationMatrix = {
    //         {cos(radians), sin(radians)},
    //         {-sin(radians), cos(radians)}
    //     };
    
    //     // Rotate points
    //     mat rotatedPoints = rotationMatrix * points;
    
    //     // Use the coordinates after rotation
    //     double newX1 = rotatedPoints(1, 0) + robotPositionX;
    //     double newY1 = rotatedPoints(0, 0) + robotPositionY;
    
    //     // For other points, you can similarly use newX and newY values
    //     // cout << "Rotated X Coordinates: " << newX1 << endl;
    //     // cout << "Rotated Y Coordinates: " << newY1 << endl;
    //     objecPositionX_ = newX1;
    //     objecPositionY_ = newY1;
    // }

    

    void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        robotPositionX_ = msg->pose.position.x;
        robotPositionY_ = msg->pose.position.y;
        robotRotationZ_ = msg->pose.orientation.z;
        robotRotationW_ = msg->pose.orientation.w;
        // robotRotationYaw_ = baha_calculation(robotRotationZ_, robotRotationW_);
        // rotateCoordinates(robotPositionX_, robotPositionY_,  robotRotationYaw_);
        robot_x.push_back(robotPositionX_);
        robot_y.push_back(robotPositionY_);

        // Timestamp the robot_x and robot_y and store it in a list
        auto timestamp = std::chrono::system_clock::now();
        const std::time_t t_c = std::chrono::system_clock::to_time_t(timestamp);
        robot_x_timestamp.push_back(t_c);
        robot_y_timestamp.push_back(t_c);
        if (robot_x.size() > 3)
        {
            RCLCPP_INFO(this->get_logger(), "Last 3 Robot Pose: (%f, %f), (%f, %f), (%f, %f)",
                robot_x[0], robot_y[0], robot_x[1], robot_y[1], robot_x[2], robot_y[2]);
            std::cout << "Last 3 Timestamps for Robot Pose: " << std::ctime(&robot_x_timestamp[0]) << ", " << std::ctime(&robot_x_timestamp[1]) << ", " << std::ctime(&robot_x_timestamp[2]) << std::endl;
            std::cout << "Last 3 Timestamps for Distance : " << std::ctime(&lastDistance1_timestamp[0]) << ", " << std::ctime(&lastDistance2_timestamp[0]) << ", " << std::ctime(&lastDistance3_timestamp[0]) << std::endl;
            if (calculate == 1 && 0 == (std::any_of(lastDistance1.begin(), lastDistance1.end(), [](double distance) { return distance == 0; }) ||
                std::any_of(lastDistance2.begin(), lastDistance2.end(), [](double distance) { return distance == 0; }) ||
                std::any_of(lastDistance3.begin(), lastDistance3.end(), [](double distance) { return distance == 0; })))
            {
                std::vector<Point> anchors = {{robot_x[0], robot_y[0]}, {robot_x[1], robot_y[1]}, {robot_x[2], robot_y[2]}};
                std::vector<double> distances = {lastDistance1[0], lastDistance2[0], lastDistance3[0]};    
                Point position = trilateration(anchors, distances);

                // std::cout << "Trilateration result: (" << position.x << ", " << position.y << ")" << std::endl;
            }
            robot_x.erase(robot_x.begin());
            robot_y.erase(robot_y.begin());
            robot_x_timestamp.erase(robot_x_timestamp.begin());
            robot_y_timestamp.erase(robot_y_timestamp.begin());
        }
    }

    void uwb_data_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string uwbData = msg->data;
        std::string id = uwbData.substr(0, 4);
        double distance = std::stod(uwbData.substr(4, 5)) / 1000;
        double rxPower = std::stod(uwbData.substr(9)) / 1000 * -1;

        auto timestamp = std::chrono::system_clock::now();
        const std::time_t t_c = std::chrono::system_clock::to_time_t(timestamp);
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
            recentMeasurements.push_back(std::vector<double>{distance}); // Initialize the recent measurements buffer for this anchor
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
                double mean = calculateMean(recentMeasurements[index]);
                double stdDev = calculateStdDev(recentMeasurements[index], mean);
    
                // Optionally filter out measurements
                recentMeasurements[index].erase(
                    std::remove_if(recentMeasurements[index].begin(), recentMeasurements[index].end(),
                                   [mean, stdDev](double x) { return std::abs(x - mean) > M * stdDev; }),
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

            lastDistance1_timestamp.push_back(t_c);
            lastDistance2_timestamp.push_back(t_c);
            lastDistance3_timestamp.push_back(t_c);
        }
        lastDistance1_timestamp.erase(lastDistance1_timestamp.begin());
        lastDistance2_timestamp.erase(lastDistance2_timestamp.begin());
        lastDistance3_timestamp.erase(lastDistance3_timestamp.begin());
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
