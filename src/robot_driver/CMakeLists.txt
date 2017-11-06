cmake_minimum_required(VERSION 2.8.3)
project(robot_driver)

find_package(
    catkin REQUIRED COMPONENTS roscpp ecl_build ecl_devices
)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp ecl_devices
    DEPENDS
)

ecl_enable_cxx11_compiler()

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(servo_driver src/driver/servo_driver.cpp)
target_link_libraries(servo_driver ${catkin_LIBRARIES})
