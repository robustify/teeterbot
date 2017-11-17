cmake_minimum_required(VERSION 2.8.3)
project(teeterbot_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/MotorSpeedControl.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# PID speed control of Teeterbot's motors
add_executable(motor_speed_control_node
  src/motor_speed_control_node.cpp
  src/MotorSpeedController.cpp
)
add_dependencies(motor_speed_control_node
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(motor_speed_control_node
  ${catkin_LIBRARIES}
)