cmake_minimum_required(VERSION 2.8.3)
project(robot_driver)

find_package(
    catkin REQUIRED COMPONENTS roscpp ecl_build ecl_devices ecl_sigslots
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp ecl_devices ecl_sigslots
)

ecl_enable_cxx11_compiler()

include_directories(${catkin_INCLUDE_DIRS} include)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_library(${PROJECT_NAME} src/servo_driver.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(servo_driver src/servo_driver.cpp)
target_link_libraries(servo_driver ${catkin_LIBRARIES})
