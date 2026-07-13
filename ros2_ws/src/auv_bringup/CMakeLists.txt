cmake_minimum_required(VERSION 3.8)
project(auv_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_saver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
