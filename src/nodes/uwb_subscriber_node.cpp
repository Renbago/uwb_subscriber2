#include "uwb_subscriber.h"

using namespace uwb_subscriber;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto uwb_node = rclcpp::Node::make_shared("uwb_subscriber");
  try {
    RCLCPP_INFO(uwb_node->get_logger(), "[UWB subscriber]: Initializing node");
    uwbSubscriber ot(uwb_node, uwb_node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(uwb_node);
    executor.spin();
  }
  catch (const char* s) {
    RCLCPP_FATAL_STREAM(uwb_node->get_logger(), "[UWB subscriber]: "  << s);
  }
  catch (const std::exception &exc) {
    auto eptr = std::current_exception(); // capture
    RCLCPP_FATAL_STREAM(uwb_node->get_logger(), "[UWB subscriber]: " << exc.what());
  }
  catch (...){
    RCLCPP_FATAL_STREAM(uwb_node->get_logger(), "[UWB subscriber]: Unknown error");
  }
  rclcpp::shutdown();
  return 0;
}