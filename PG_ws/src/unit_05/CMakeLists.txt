cmake_minimum_required(VERSION 2.8.3)
project(unit_05)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
)

find_package(OpenCV REQUIRED)
catkin_package(
CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS}
)


add_executable(guess_01 src/guess_01.cpp)
target_link_libraries (guess_01 ${OpenCV_LIBS} ${catkin_LIBRARIES})




