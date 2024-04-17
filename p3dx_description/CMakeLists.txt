cmake_minimum_required(VERSION 3.5)
project(p3dx_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY launch meshes urdf
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
