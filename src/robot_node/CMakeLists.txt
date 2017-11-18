cmake_minimum_required(VERSION 2.8.3)
project(robot_node)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib
                ecl_exceptions ecl_sigslots ecl_streams ecl_threads
                controller_manager nav_msgs realtime_tools tf urdf
                robot_driver
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES robot_ros robot_nodelet
   CATKIN_DEPENDS roscpp nodelet pluginlib
                  ecl_exceptions ecl_sigslots ecl_streams ecl_threads
                  controller_manager nav_msgs realtime_tools tf urdf
                  robot_driver
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(robot_ros src/robot_ros.cpp src/slot_callbacks.cpp src/robot.cpp)
add_dependencies(robot_ros robot_driver)
target_link_libraries(robot_ros ${catkin_LIBRARIES})

add_library(robot_nodelet src/robot_nodelet.cpp)
add_dependencies(robot_nodelet robot_ros)
target_link_libraries(robot_nodelet robot_ros)

