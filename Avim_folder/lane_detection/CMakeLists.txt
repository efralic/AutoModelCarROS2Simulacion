cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ── Dependencias ──────────────────────────────────────────────────────────────
# ROS 1: find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge ...)
# ROS 2: find_package por separado para cada paquete
find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclpy              REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(cv_bridge          REQUIRED)
find_package(image_transport    REQUIRED)
find_package(OpenCV             REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# ── Ejecutable C++: lane_detection ───────────────────────────────────────────
# Sustituye al lane_detection.cpp original que detectaba un solo blob blanco.
# Ahora incluye:
#   • ROI dinámico
#   • Máscara HSV ∩ LAB con CLAHE (anti-brillos)
#   • Bird's-eye view (vista cenital)
#   • Histograma + ventanas deslizantes
#   • Ajuste polinómico de grado 2
#   • Suavizado temporal (promedio móvil)
#   • Cálculo de error lateral con fallback
add_executable(lane_detection src/lane_detection.cpp)

ament_target_dependencies(lane_detection
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# OpenCV se enlaza como librería del sistema
target_link_libraries(lane_detection ${OpenCV_LIBRARIES})

# ── Instalar ejecutable C++ ───────────────────────────────────────────────────
# ROS 2 requiere install() explícito para que ros2 run lo encuentre
install(TARGETS lane_detection
  DESTINATION lib/${PROJECT_NAME}
)

# ── Instalar script Python (stop_sign_detector.py) ────────────────────────────
# ROS 1: catkin_install_python(PROGRAMS src/stop_sign_detector.py ...)
# ROS 2: install(PROGRAMS ...)
install(PROGRAMS
  src/stop_sign_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Cerrar paquete ────────────────────────────────────────────────────────────
ament_package()
