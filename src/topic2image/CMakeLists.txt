cmake_minimum_required(VERSION 3.0.2)
project(topic2image)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions     
  pcl_ros
  geometry_msgs
  laser_geometry
  sensor_msgs
  message_generation
  roslaunch
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topic2image
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
add_executable(topic2image src/main.cpp src/points_image.cpp)

target_link_libraries(topic2image ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})



