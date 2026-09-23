cmake_minimum_required(VERSION 3.8)
project(object_detection_parking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ── Dependencias ──────────────────────────────────────────────────────────────
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge     REQUIRED)
find_package(OpenCV        REQUIRED)

# ── Mensaje custom: importado desde el paquete object_detection ───────────────
# ROS 1: find_package(object_detection REQUIRED) + include_directories(...)
# ROS 2: find_package(object_detection REQUIRED) — suficiente porque
#         object_detection exporta su interfaz con ament_export_dependencies
find_package(object_detection REQUIRED)

# ── Ejecutable ────────────────────────────────────────────────────────────────
add_executable(object_detection_parking_node src/object_detection_parking.cpp)

ament_target_dependencies(object_detection_parking_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  object_detection        # ← da acceso a object_detection::msg::PointsObjects
)

target_link_libraries(object_detection_parking_node
  ${OpenCV_LIBRARIES}
)

# ── Instalación ───────────────────────────────────────────────────────────────
install(TARGETS object_detection_parking_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
