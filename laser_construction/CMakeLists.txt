cmake_minimum_required(VERSION 3.0.2)
project(laser_construction)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
)


catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(laser_construction src/laser_construction.cc)
target_link_libraries(laser_construction 	
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${PCL_LIBRARIES}
)

