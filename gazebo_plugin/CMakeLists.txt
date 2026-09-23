cmake_minimum_required(VERSION 3.8)
project(gazebo_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ignition-transport/msgs vienen con la instalación de Gazebo 11; se
# comprueban aquí igual que en el CMakeLists.txt original (ROS 1), solo a
# modo de verificación temprana.
find_package(ignition-msgs1 QUIET)
find_package(ignition-msgs5 QUIET)
find_package(ignition-transport4 QUIET)
find_package(ignition-transport8 QUIET)

include_directories(include)
link_directories(${gazebo_dev_LIBRARY_DIRS})

# ---- autonomos_plugin: control de dirección/velocidad (usa ROS 2) --------
add_library(autonomos_plugin SHARED src/autonomos_plugin.cpp)
target_include_directories(autonomos_plugin PUBLIC include)
ament_target_dependencies(autonomos_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
)

# ---- parking_lot: generador de escenas de estacionamiento (solo Gazebo) --
add_library(parking_lot SHARED src/parking_lot.cc)
target_include_directories(parking_lot PUBLIC include)
ament_target_dependencies(parking_lot "gazebo_dev")

install(
  TARGETS autonomos_plugin parking_lot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(autonomos_plugin parking_lot)
ament_package()
