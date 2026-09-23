cmake_minimum_required(VERSION 3.8)
project(bring_up)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/spawn_no_obstacles.py
  scripts/spawn_static_obstacles.py
  scripts/spawn_dynamic_obstacles.py
  scripts/spawn_parking.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
