# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.16.3)
project(tinker_navigation)

## Find catkin and any catkin packages
find_package(catkin PCL REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs)

include_directories(
  include ${catkin_INCLUDE_DIRS}
#  include /home/hazel/pcl/common/include
  include /usr/include/eigen3
  include /usr/include/pcl-1.10
  include /opt/ros/noetic/share/
  ${PCL_INCLUDE_DIRS}
)

catkin_package()

# add_executable(projectionNode src/pcl/proj_1.cpp)
# target_link_libraries(projectionNode
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )