cmake_minimum_required(VERSION 2.8.3)
project(laser_ekf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf
  tf_conversions
  libpointmatcher_ros
  roslib
)

add_definitions(-std=c++11 -O2 -g -DNDEBUG)

catkin_package (
#	CATKIN_DEPENDS roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

add_executable(ekf_simple src/ekf_simple.cpp)
target_link_libraries(ekf_simple
${catkin_LIBRARIES}
)

add_executable(sliding_mapper_double src/sliding_mapper_double.cpp)
target_link_libraries(sliding_mapper_double
${catkin_LIBRARIES}
)

add_executable(odom_zh src/odom_zh.cpp)
target_link_libraries(odom_zh
${catkin_LIBRARIES}
)

#add_executable(odom_imu src/odom_imu.cpp)
#target_link_libraries(odom_imu
#${catkin_LIBRARIES}
#)

add_executable(ekf_complex src/ekf_complex.cpp)
target_link_libraries(ekf_complex
${catkin_LIBRARIES}
)


