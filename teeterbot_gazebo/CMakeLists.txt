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
  cfg/MotorControl.cfg
)

add_service_files(
  FILES
    NudgeTeeterbot.srv
)

generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Gazebo plugin to manage Teeterbot simulation
add_library(${PROJECT_NAME}_plugin
  src/ControllerInterfacePlugin.cpp
  src/DcMotorSim.cpp
  src/MotorController.cpp
)
add_dependencies(${PROJECT_NAME}_plugin
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
