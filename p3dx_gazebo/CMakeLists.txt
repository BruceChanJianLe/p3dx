cmake_minimum_required(VERSION 3.5)
project(p3dx_gazebo)

find_package(ament_cmake REQUIRED)
find_package(p3dx_description REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

install(DIRECTORY launch models worlds
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
