cmake_minimum_required(VERSION 2.8.3)
project(asr_mild_base_driving)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
)

find_package(SDL REQUIRED)

find_package(Boost)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs tf nav_msgs
	#DEPENDS SDL Boost
	DEPENDS Boost
)

include_directories(
	include
	${SDL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
)



add_executable(asr_mild_base_driving 
	src/main.cpp 
	src/BaseController.cpp 
	src/CanListener.cpp
)


target_link_libraries(asr_mild_base_driving

	${catkin_LIBRARIES}
	${SDL_LIBRARIES}
)
