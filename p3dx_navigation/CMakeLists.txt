cmake_minimum_required(VERSION 3.5)
project(p3dx_navigation)

find_package(ament_cmake REQUIRED)
find_package(navigation2 REQUIRED)

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})

ament_package()
