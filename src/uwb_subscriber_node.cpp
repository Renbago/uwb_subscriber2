#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <chrono>

std::vector<std::string> anchor_addr_list;
std::vector<float> distance_list;
std::vector<float> rx_power_list;
std::vector<float> robot_x;
std::vector<float> robot_y;
std::vector<std::chrono::steady_clock::time_point> last_occurrence_list;

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
    void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Robot Pose - Position: [X: %.2f, Y: %.2f, Z: %.2f]",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
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
            rx_power_list.push_back(rxPower);
            last_occurrence_list.push_back(std::chrono::steady_clock::now());
        }
        else 
        {    
            auto index = it - anchor_addr_list.begin();
            distance_list[index] = distance;
            rx_power_list[index] = rxPower;
            last_occurrence_list[index] = std::chrono::steady_clock::now();
        }
        for (size_t i = 0; i < anchor_addr_list.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Anchor %s - Distance: %f, RX Power: %f",
                        anchor_addr_list[i].c_str(), distance_list[i], rx_power_list[i]);
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
                rx_power_list.erase(rx_power_list.begin() + i);
                last_occurrence_list.erase(last_occurrence_list.begin() + i);
                i--; // Adjust the index after erasing an element
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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
