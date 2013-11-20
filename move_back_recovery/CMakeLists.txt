cmake_minimum_required(VERSION 2.8.3)
project(move_back_recovery)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  eigen
  nav_core
  pluginlib
  roscpp
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES move_back_recovery
 CATKIN_DEPENDS pluginlib roscpp
)

add_library(move_back_recovery
  src/move_back_recovery.cpp
)

target_link_libraries(move_back_recovery
  ${catkin_LIBRARIES}
)

install(TARGETS move_back_recovery
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  move_back_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
