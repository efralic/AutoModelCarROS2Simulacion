cmake_minimum_required(VERSION 3.8)
project(control)

# Compilar con C++17 (recomendado para ROS 2; mínimo C++14)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ── Dependencias ──────────────────────────────────────────────────────────────
# ROS 1: find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ...)
# ROS 2: find_package por separado para cada paquete
find_package(ament_cmake      REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(object_detection  REQUIRED)   # paquete con el msg custom PointsObjects
find_package(nav_msgs          REQUIRED)   # NUEVO: /lane/left_line y /lane/right_line (Master_overtake)

# Macro helper para no repetir ament_target_dependencies en cada ejecutable
set(DEPS
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  object_detection
  nav_msgs
)

# NUEVO: cabecera overtake_core.hpp (lógica del rebase, sin ROS)
include_directories(include)

# ── Ejecutables ───────────────────────────────────────────────────────────────
# ROS 1: add_executable + target_link_libraries(target ${catkin_LIBRARIES})
# ROS 2: add_executable + ament_target_dependencies(target ...)

add_executable(Master src/Master.cpp)
ament_target_dependencies(Master ${DEPS})

add_executable(Master_parking src/Master_parking_bateria.cpp)
ament_target_dependencies(Master_parking ${DEPS})

# NUEVO: Master de rebase robusto (lazo cerrado)
add_executable(Master_parking_v2 src/Master_parking_v2.cpp)
ament_target_dependencies(Master_parking_v2 ${DEPS})

add_executable(Master_overtake src/Master_overtake.cpp)
ament_target_dependencies(Master_overtake ${DEPS})

# ── Instalación de ejecutables ────────────────────────────────────────────────
# ROS 2 requiere instalar explícitamente para que ros2 run los encuentre
install(TARGETS
  Master
  Master_parking
  Master_overtake
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# ── Cerrar el paquete ─────────────────────────────────────────────────────────
# ROS 1: catkin_package() al principio
# ROS 2: ament_package() al final
install(TARGETS Master_parking_v2 DESTINATION lib/${PROJECT_NAME})

ament_package()
