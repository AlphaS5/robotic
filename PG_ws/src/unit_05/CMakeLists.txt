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


catkin_install_python(PROGRAMS src/guess_03.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/image_converter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




add_executable(guess_01 src/guess_01.cpp)
target_link_libraries (guess_01 ${OpenCV_LIBS} ${catkin_LIBRARIES})


add_executable(guess_02 src/guess_02.cpp)
target_link_libraries (guess_02 ${OpenCV_LIBS} ${catkin_LIBRARIES})


#add_executable(guess_03 src/guess_03.py)
#target_link_libraries (guess_03 ${OpenCV_LIBS} ${catkin_LIBRARIES})




