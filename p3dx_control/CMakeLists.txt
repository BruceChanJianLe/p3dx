cmake_minimum_required(VERSION 3.10)
project(p3dx_control)

find_package(ament_cmake REQUIRED)
find_package(robot_localization REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
