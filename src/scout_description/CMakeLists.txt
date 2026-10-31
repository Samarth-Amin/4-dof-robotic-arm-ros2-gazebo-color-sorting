cmake_minimum_required(VERSION 3.16)
project(scout_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz config worlds maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()