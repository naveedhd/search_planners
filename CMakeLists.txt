cmake_minimum_required(VERSION 2.8.3)
project(search_planners)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  catch_ros
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    pluginlib
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_subdirectory(src)
add_subdirectory(test)
