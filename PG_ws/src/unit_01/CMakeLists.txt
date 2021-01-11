# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(unit_01)

# Find and get all the information about the roscpp package
find_package(roscpp REQUIRED)

# Find Catkin
find_package(catkin REQUIRED)
# Declare this project as a catkin package
catkin_package()

# Add the local headers and the headers from roscpp
include_directories(include ${roscpp_INCLUDE_DIRS})


# Define executables target called robo_XX
# unit 01: robo_A, robo_B, robo_C, robo_D
add_executable(robo_A src/robo_A.cpp)
target_link_libraries(robo_A ${roscpp_LIBRARIES})
#
add_executable(robo_B src/robo_B.cpp)
target_link_libraries(robo_B ${roscpp_LIBRARIES})
#
add_executable(robo_C src/robo_C.cpp)
target_link_libraries(robo_C ${roscpp_LIBRARIES})
#
add_executable(robo_D src/robo_D.cpp)
target_link_libraries(robo_D ${roscpp_LIBRARIES})
#
add_executable(robo_XYZ src/robo_XYZ.cpp)
target_link_libraries(robo_XYZ ${roscpp_LIBRARIES})
#


#thats all folks

