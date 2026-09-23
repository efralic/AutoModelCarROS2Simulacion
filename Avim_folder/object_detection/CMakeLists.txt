cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ── Dependencias ──────────────────────────────────────────────────────────────
# ROS 1: find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs ...)
# ROS 2: find_package por separado + rosidl para generar el msg custom
find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(std_msgs       REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(cv_bridge      REQUIRED)
find_package(OpenCV         REQUIRED)

# ── Mensaje custom PointsObjects ──────────────────────────────────────────────
# ROS 1: add_message_files + generate_messages(DEPENDENCIES geometry_msgs)
# ROS 2: rosidl_generate_interfaces — genera el código C++/Python del .msg
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointsObjects.msg"
  DEPENDENCIES geometry_msgs
)

# IMPORTANTE en ROS 2: el ejecutable que usa el msg custom necesita
# declarar la dependencia con rosidl_get_typesupport_target
# (reemplaza el add_dependencies catkin de ROS 1)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── Ejecutable ────────────────────────────────────────────────────────────────
add_executable(object_detection_node src/object_detection.cpp)

ament_target_dependencies(object_detection_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

# Enlazar con el soporte del msg custom generado
target_link_libraries(object_detection_node
  "${cpp_typesupport_target}"
  ${OpenCV_LIBRARIES}
)

# ── Instalación ───────────────────────────────────────────────────────────────
install(TARGETS object_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Exportar el paquete para que otros paquetes (control, etc.) 
# puedan usar el msg PointsObjects
ament_export_dependencies(rosidl_default_runtime)

ament_package()
