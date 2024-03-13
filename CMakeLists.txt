cmake_minimum_required(VERSION 3.8)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
project(uwb_subscriber)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost 1.54.0 REQUIRED system)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# ament_target_dependencies(
# uwb_subscriber_node
# rclcpp
# std_msgs
# geometry_msgs
# Armadillo
# )

set(PROJECT_INCLUDE_DIRS include)
set(PROJECT_DEPENDS "rclcpp" "std_msgs" "std_srvs" "tf2_sensor_msgs" "geometry_msgs" "visualization_msgs" "nav2_msgs"
     "tf2_geometry_msgs" "tf2_ros" )

include_directories(include ${PROJECT_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Build libs
add_library(uwb_subscriber src/uwb_subscriber.cpp)
target_link_libraries(uwb_subscriber ${ARMADILLO_LIBRARIES})
ament_target_dependencies(uwb_subscriber ${PROJECT_DEPENDS} tf2 tf2_ros tf2_sensor_msgs)
# allow the project to link against its own messages
target_link_libraries(uwb_subscriber "${cpp_typesupport_target}")

add_executable(uwb_subscriber_node src/nodes/uwb_subscriber_node.cpp)
target_link_libraries(uwb_subscriber_node uwb_subscriber)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS uwb_subscriber  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  uwb_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()