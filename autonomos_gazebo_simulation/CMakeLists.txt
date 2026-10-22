cmake_minimum_required(VERSION 3.8)
project(autonomos_gazebo_simulation)

find_package(ament_cmake REQUIRED)

# Este paquete no compila código: solo empaqueta mundos y modelos de Gazebo.
install(DIRECTORY
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Agrega share/autonomos_gazebo_simulation/models a GAZEBO_MODEL_PATH cada
# vez que se hace `source install/setup.bash` (necesario para que Gazebo
# resuelva las referencias model://... dentro de los .sdf/.world). La ruta
# se resuelve aquí, en tiempo de build (CMAKE_INSTALL_PREFIX), y se sustituye
# como texto literal dentro del .sh.in -- no depende de variables de shell
# en tiempo de ejecución.
set(AUTONOMOS_GAZEBO_MODELS_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/autonomos_gazebo_simulation.sh.in")

ament_package()
