# AutomodelCar2026_ros2 — Simulación Gazebo del AVIM (ROS 2 Humble)

Puerto a ROS 2 Humble / Gazebo Classic 11 del entorno de simulación del
proyecto original [`AutomodelCar2026`](https://github.com/efralic/AutomodelCar2026)
(ROS 1). Este repo **solo trae el entorno de simulación** (mundo, modelo del
carrito, plugin de control): la percepción y el control siguen viviendo en
[`AutoModelCarROS2`](https://github.com/efralic/AutoModelCarROS2), que se
clona aparte como paquetes hermanos en el mismo workspace.

## Qué cambió respecto al repo original

- Se **quitó el LIDAR** del modelo `AutoNOMOS_mini` (sensor `laser` tipo
  `ray`, tópico `/scan`). El modelo ya traía también una cámara de
  profundidad simulada tipo Kinect — es la que se usa ahora, con FOV y
  rango ajustados a la Nuwa-HP60C real (73.8°, 0.2–4 m).
- El plugin de esa cámara se actualizó de `libgazebo_ros_openni_kinect.so`
  (ROS 1) a `libgazebo_ros_camera.so` (ROS 2).
- El paquete `Gazebo_plugin` (ROS 1) se convirtió en `gazebo_plugin`
  (ROS 2): `autonomos_plugin` se portó de `roscpp` a `rclcpp` +
  `gazebo_ros::Node`; `parking_lot` se copió tal cual (no depende de ROS).
  Se quitó `marker.cc` (ejemplo de tutorial sin usar).
- `bring_up` se reescribió: los `.launch` (XML) pasaron a `.launch.py`, y
  los scripts `testN_start.py` (rospy) pasaron a `spawn_*.py` (rclpy),
  usando el servicio `/spawn_entity` en vez de `/gazebo/spawn_sdf_model`.
- Se **descartó por completo `AVIM_folder`** (lane_detection, object_detection,
  control, hardware_interface viejos, basados en LIDAR): ya existen
  versiones más avanzadas de esos mismos paquetes en `AutoModelCarROS2`.
- Se quitaron los diagramas/documentación que describían la arquitectura
  vieja con LIDAR, y los launch de pistas sueltas (`curved_road`,
  `intersection`, etc.) que ya no tenían sus mundos correspondientes.

## Estructura

```
AutomodelCar2026_ros2/
├── autonomos_gazebo_simulation/   # mundos (TMR2021, TMR2021_parking) y modelos
├── gazebo_plugin/                 # autonomos_plugin (ROS2) + parking_lot
├── bring_up/                      # launch files + scripts de spawn
└── Media/                         # layout de la pista (referencia)
```

## Instalación

1. Clona este repo **y** `AutoModelCarROS2` como paquetes hermanos dentro
   de `src/` de tu workspace (ver conversación previa sobre mantenerlos
   separados del workspace del RoverLunar):

   ```bash
   mkdir -p ~/avim_ws/src && cd ~/avim_ws/src
   git clone <url-de-este-repo-nuevo> AutomodelCar2026_ros2
   git clone https://github.com/efralic/AutoModelCarROS2.git
   ```

2. Instala dependencias y compila:

   ```bash
   cd ~/avim_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

## Uso

```bash
ros2 launch bring_up sim_no_obstacles.launch.py       # Prueba 1: señales
ros2 launch bring_up sim_static_obstacles.launch.py   # Prueba 2: obstáculos estáticos
ros2 launch bring_up sim_dynamic_obstacles.launch.py  # Prueba 3: obstáculos dinámicos
ros2 launch bring_up sim_parking.launch.py            # Prueba 4: estacionamiento
```

## Pendientes / cosas a verificar tú mismo

- **Nombres de tópico de la cámara simulada**: quedaron como
  `/camera/image_raw` y `/camera/depth/image_raw` por defecto
  (`camera_name: camera` en el SDF). Confirma con `ros2 topic list` una
  vez levantada la sim y ajusta el parámetro `camera_topic`/`depth_topic`
  en los launch si el nombre real difiere.
- **`spawn_dynamic_obstacles.py`**: usa `/get_entity_state` sobre el
  modelo completo `AutoModel_Obstacle1` en vez del link específico que
  usaba la versión ROS 1 (`GetLinkState` no tiene equivalente por-link en
  ROS 2). Debería darte una pose equivalente, pero no se pudo probar en
  un Gazebo real desde aquí.
- **`config_files/rviz/mapping.rviz`** del repo original no se incluyó:
  es un formato de RViz 1 (ROS 1), incompatible con RViz2. Si lo necesitas,
  hay que rearmar esa vista guardándola desde RViz2 directamente.
- Ninguno de estos paquetes se compiló/probó en un Gazebo real desde este
  entorno (aquí no hay ROS2/Gazebo instalado) — revisa errores de
  `colcon build` la primera vez, especialmente nombres de dependencias
  (`gazebo_dev`, `gazebo_ros`) según tu instalación exacta de Humble.
