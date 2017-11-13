cmake_minimum_required(VERSION 2.8.3)
project(teeterbot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  tf
  roscpp
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(teeterbot_controller_interface
  src/ControllerInterfacePlugin.cpp
  src/DcMotorSim.cpp
)
target_link_libraries(teeterbot_controller_interface
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
