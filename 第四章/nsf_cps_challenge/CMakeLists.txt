cmake_minimum_required(VERSION 2.8.3)
project(wfov_convert)

find_package(catkin REQUIRED COMPONENTS
	darknet_ros_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
