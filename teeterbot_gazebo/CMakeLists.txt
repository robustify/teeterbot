cmake_minimum_required(VERSION 2.8.3)
project(teeterbot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  dynamic_reconfigure
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(
  cfg/MotorSpeedControl.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Gazebo plugin to manage Teeterbot simulation
add_library(teeterbot_controller_interface
  src/ControllerInterfacePlugin.cpp
  src/DcMotorSim.cpp
)
target_link_libraries(teeterbot_controller_interface
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
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
