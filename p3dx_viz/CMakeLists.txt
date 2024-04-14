cmake_minimum_required(VERSION 2.8.3)
project(p3dx_viz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz2
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
