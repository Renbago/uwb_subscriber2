cmake_minimum_required(VERSION 3.8)
project(uwb_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Armadillo REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(uwb_subscriber_node src/uwb_subscriber_node.cpp)
ament_target_dependencies(uwb_subscriber_node
rclcpp
std_msgs
geometry_msgs
Armadillo
)
include_directories(include ${ARMADILLO_INCLUDE_DIRS} )

target_link_libraries(uwb_subscriber_node ${ARMADILLO_LIBRARIES})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  uwb_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()