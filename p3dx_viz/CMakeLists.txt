cmake_minimum_required(VERSION 3.5)
project(p3dx_viz)

find_package(ament_cmake REQUIRED)
find_package(p3dx_description REQUIRED)
find_package(rviz2 REQUIRED)

install(DIRECTORY launch rviz2
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
