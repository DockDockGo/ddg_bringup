cmake_minimum_required(VERSION 3.8)
project(ddg_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ddg_multi_robot_planner REQUIRED)

install(DIRECTORY launch
  # config
  # rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()