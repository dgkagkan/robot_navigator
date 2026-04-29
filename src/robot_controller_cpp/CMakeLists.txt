cmake_minimum_required(VERSION 3.8)
project(robot_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(include)

add_library(turtle_controller_component SHARED src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller_component rclcpp rclcpp_action turtlesim geometry_msgs my_robot_interfaces rclcpp_lifecycle rclcpp_components nav_msgs)
rclcpp_components_register_nodes(turtle_controller_component "turtle_controller_cpp::TurtleControllerNode")

add_library(lifecycle_node_manager_component SHARED src/lifecycle_node_manager.cpp)
ament_target_dependencies(lifecycle_node_manager_component rclcpp rclcpp lifecycle_msgs rclcpp_components)
rclcpp_components_register_nodes(lifecycle_node_manager_component "lifecycle_node_manager_cpp::LifecycleNodeManager") 

install(TARGETS
  turtle_controller_component
  lifecycle_node_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION bin
)

ament_package()
