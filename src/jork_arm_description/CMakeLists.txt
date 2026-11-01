cmake_minimum_required(VERSION 3.16)
project(jork_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch meshes rviz worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pick_and_place_demo.py
    scripts/vision_sorting_cell.py
    scripts/check_project_consistency.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()