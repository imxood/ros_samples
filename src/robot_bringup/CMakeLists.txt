cmake_minimum_required(VERSION 2.8.3)
project(robot_bringup)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    nav_msgs
    realtime_tools
    tf
    urdf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(my_robot src/robot.cpp)
target_link_libraries(my_robot ${catkin_LIBRARIES})
