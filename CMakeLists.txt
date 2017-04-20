cmake_minimum_required(VERSION 2.8.3)
project(masked_image)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(masked_image src/masked_image.cpp)

target_link_libraries(masked_image
  ${catkin_LIBRARIES}
)

