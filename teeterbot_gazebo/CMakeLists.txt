cmake_minimum_required(VERSION 3.5)
project(teeterbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

include_directories(include)

# add_library(TeeterbotInterfacePlugin SHARED
#   src/TeeterbotInterfacePlugin.cpp
# )
# target_link_libraries(TeeterbotInterfacePlugin
#   gz-sim::gz-sim
#   gz-plugin::register
# )

# ament_export_libraries(TeeterbotInterfacePlugin)
# install(
#   TARGETS TeeterbotInterfacePlugin
#   DESTINATION lib
# )

install(
  DIRECTORY
    config
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
