cmake_minimum_required(VERSION 3.8)
project(autonomos_gazebo_simulation)

find_package(ament_cmake REQUIRED)

# Este paquete no compila código: solo empaqueta mundos y modelos de Gazebo.
install(DIRECTORY
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
