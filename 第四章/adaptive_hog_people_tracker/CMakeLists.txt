cmake_minimum_required(VERSION 2.8.3)
project(adaptive_hog_people_tracker)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
   FILES
   BoundingBox.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########
include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(HOGDetector
	src/HOGDetector.cpp
)

add_library(LinearSVM
	src/LinearSVM.cpp
)

add_executable(person_tracker src/person_tracker.cpp)
target_link_libraries(person_tracker ${OpenCV_LIBS} ${catkin_LIBRARIES} HOGDetector LinearSVM)

