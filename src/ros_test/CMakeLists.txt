cmake_minimum_required(VERSION 2.8.3)
project(ros_test)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib ecl_threads)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES test_nodelet
   CATKIN_DEPENDS roscpp nodelet pluginlib ecl_threads
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(test_nodelet src/test_nodelet.cpp)
target_link_libraries(test_nodelet ${catkin_LIBRARIES})

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
