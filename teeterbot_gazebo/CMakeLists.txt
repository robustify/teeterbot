cmake_minimum_required(VERSION 3.5)
project(teeterbot_gazebo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})


rosidl_generate_interfaces(${PROJECT_NAME}
  srv/NudgeTeeterbot.srv
)
ament_export_dependencies(rosidl_default_runtime)

# Gazebo plugin to manage Teeterbot simulation
add_library(${PROJECT_NAME}_plugin SHARED
  src/ControllerInterfacePlugin.cpp
)
ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp
  tf2_ros
  gazebo_ros
  example_interfaces
)
rosidl_target_interfaces(${PROJECT_NAME}_plugin
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
