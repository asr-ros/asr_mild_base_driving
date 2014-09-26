cmake_minimum_required(VERSION 2.8.3)
project(can)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
)


catkin_package(INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(  ${catkin_INCLUDE_DIRS})

add_executable(can src/can.cpp src/BaseController.cpp src/CanListener.cpp)
find_package(Boost)


target_link_libraries(can
   ${catkin_LIBRARIES}
)
