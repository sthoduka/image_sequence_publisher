cmake_minimum_required(VERSION 2.8.3)
project(image_sequence_publisher)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    cv_bridge
    image_transport
)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

### EXECUTABLES ###############################################
add_executable(image_sequence_publisher
  ros/src/image_sequence_publisher_node.cpp
)
target_link_libraries(image_sequence_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
