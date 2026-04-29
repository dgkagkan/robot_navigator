cmake_minimum_required(VERSION 3.8)
project(robot_controller_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_robot_interfaces REQUIRED)
include_directories(include)


add_executable(teleop_keyboard src/teleop_keyboard.cpp)
ament_target_dependencies(teleop_keyboard rclcpp my_robot_interfaces)



install(TARGETS
  teleop_keyboard
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
