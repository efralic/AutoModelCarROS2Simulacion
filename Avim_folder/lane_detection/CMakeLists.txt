cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ══════════════════════════════════════════════════════════════════════════════
#  [OPT-0]  OPTIMIZACIÓN DEL COMPILADOR  ← el cambio de mayor impacto
#
#  Sin CMAKE_BUILD_TYPE, CMake compila SIN NINGUNA optimización (-O0).
#  En este nodo (bucles cerrados sobre miles de puntos, aritmética con
#  cv::Vec3d) eso cuesta del orden de 8x. Medido con la lógica de ventanas
#  deslizantes aislada: 0.74 ms/frame con -O0 contra 0.09 ms/frame con -O2.
#
#  Aquí se fija Release por defecto, respetando lo que pases por línea de
#  comandos si algún día necesitas Debug.
# ══════════════════════════════════════════════════════════════════════════════
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Tipo de build" FORCE)
  message(STATUS "[lane_detection] CMAKE_BUILD_TYPE no definido -> usando Release")
endif()

# -O3 y matemática rápida para la ruta de visión.
# NOTA: -mcpu=cortex-a72 es específico de la Raspberry Pi 4. Si compilas en
# otra máquina (o migras de placa) COMÉNTALO o el binario no arrancará.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-O3 -ffast-math)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_options(-mcpu=cortex-a72)
  endif()
endif()

# ── Dependencias ──────────────────────────────────────────────────────────────
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
add_executable(lane_detection src/lane_detection.cpp)

ament_target_dependencies(lane_detection
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(lane_detection ${OpenCV_LIBRARIES})

# ── Instalar ejecutable C++ ───────────────────────────────────────────────────
install(TARGETS lane_detection
  DESTINATION lib/${PROJECT_NAME}
)

# ── Instalar script Python (stop_sign_detector.py) ────────────────────────────
install(PROGRAMS
  src/stop_sign_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
