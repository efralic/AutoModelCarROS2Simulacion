# 🚗 AVIM – Autonomous Vehicle Interface Manager

> Sistema de conducción autónoma para el **TMR 2026** desarrollado sobre ROS 2 Humble.  
> Raspberry Pi 4B + ESP32 (micro-ROS) + Arducam IMX219 (CSI) + Nuwa-HP60C (USB)

---

## 📋 Tabla de contenido

- [Arquitectura del sistema](#arquitectura-del-sistema)
- [Hardware requerido](#hardware-requerido)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Instalación rápida](#instalación-rápida)
- [Uso](#uso)
- [Pruebas individuales](#pruebas-individuales)
- [Tópicos ROS 2](#tópicos-ros-2)
- [Dependencias](#dependencias)
- [Notas importantes](#notas-importantes)

---

## Arquitectura del sistema

```
┌──────────────────────────────────────────────────────────────────────┐
│                     RASPBERRY PI 4B  (ROS 2 Humble)                  │
│                                                                      │
│  ┌──────────────┐  ┌───────────────────────────────────────────┐    │
│  │lane_detection│  │         control (Masters)                  │    │
│  │(Arducam CSI) │  │  Master.cpp          Master_static.cpp     │    │
│  └──────────────┘  │  Master_parking_bateria.cpp                │    │
│  ┌──────────────┐  │  Master_parking_paralelo.cpp               │    │
│  │stop_sign     │  └───────────────────────────────────────────┘    │
│  │detector      │                                                    │
│  └──────────────┘  ┌───────────────────────────────────────────┐    │
│                    │       hardware_interface (Pi)              │    │
│  ┌──────────────┐  │  motor_driver.py    (I2C GPIO 2/3)         │    │
│  │object_detect.│  │  ultrasonic_sensors.py (pigpio GPIO)       │    │
│  │(Nuwa-HP60C)  │  │  teleop_controller.py  (PS4)               │    │
│  └──────────────┘  └───────────────────────────────────────────┘    │
│                          │ USB Serial (/dev/ttyUSB0)                 │
│              micro-ROS Agent ←─────────────────────────────────────┐│
└────────────────────────────────────────────────────────────────────┘│
                                                                       │
┌──────────────────────────────────────────────────────────────────── ┘
│                   ESP32  (micro-ROS) – solo LEDs                     │
│                                                                      │
│         ┌──────────────────────────────────┐                        │
│         │        lights_controller         │                        │
│         │  GPIO 14 → LED blinker izq       │                        │
│         │  GPIO 15 → LED blinker der       │                        │
│         │  GPIO 25 → LED señal izq         │                        │
│         │  GPIO 26 → LED señal der         │                        │
│         └──────────────────────────────────┘                        │
└──────────────────────────────────────────────────────────────────────┘

Cámaras:
  Arducam IMX219 (CSI) ──→ camera_ros ──→ /camera/image_raw → lane_detection
  Nuwa-HP60C (USB)     ──→ ascamera   ──→ /ascamera_hp60c/camera_publisher/...
                                          rgb0/image        → stop_sign_detector
                                          depth0/image_raw  → object_detection
                                          depth0/points     → (point cloud 3D)
```

---

## Hardware requerido

| Componente | Modelo | Conexión |
|---|---|---|
| Computadora principal | Raspberry Pi 4B — 4 GB RAM | — |
| Microcontrolador | ESP32 DevKitC | USB Serial a Pi |
| Cámara de carril | Arducam IMX219 B0394 | CSI (ribbon cable) |
| Cámara de profundidad | Yahboom/Angstrong Nuwa-HP60C | USB 3.0 |
| Módulo de motores | Encoder Motor Module I2C 0x34 | I2C Pi GPIO 2 (SDA) / 3 (SCL) |
| Servo de dirección | Servo estándar 5V | Pi GPIO 13 (pigpio PWM) |
| Sensores ultrasonido | HC-SR04 × 4 | GPIO de la Pi (via pigpio) |
| Control remoto | PS4 DualShock | USB/Bluetooth a Pi |
| Almacenamiento | microSD 32 GB Clase 10+ | — |

### Pinout ultrasonidos (GPIO BCM de la Pi)

| Sensor | TRIG | ECHO |
|---|---|---|
| Front | GPIO 5 | GPIO 6 |
| Back | GPIO 16 | GPIO 20 |
| Left | GPIO 21 | GPIO 26 |
| Right | GPIO 19 | GPIO 12 |

### Pinout ESP32 (solo LEDs)

| Función | GPIO |
|---|---|
| LED Blinker Izquierdo | 14 |
| LED Blinker Derecho | 15 |
| LED Señal Izquierda | 25 |
| LED Señal Derecha | 26 |

### Especificaciones cámara de profundidad Nuwa-HP60C

| Parámetro | Valor |
|---|---|
| FOV horizontal | 73.8° |
| Rango efectivo | 0.2 – 4.0 m |
| Tecnología | Luz estructurada binocular |
| Resolución (color y depth) | 640 × 480 |
| FPS | 25 |
| Formato depth | 16UC1 (uint16, milímetros) |
| Conexión | USB 3.0 (ID 3482:6723 NOVATEK) |

---

## Estructura del repositorio

```
avim_ros2/
├── README.md
├── .gitignore
│
├── avim/                          # Launch files principales
│   ├── launch/
│   │   ├── tmr2026_main.launch.py
│   │   ├── navigation_without_obstacles.launch.py
│   │   ├── navigation_with_static_obstacles.launch.py
│   │   ├── navigation_with_dynamic_obstacles.launch.py
│   │   ├── parking.launch.py
│   │   └── teleop.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
│
├── control/                       # Nodos maestros de control (C++)
│   ├── src/
│   │   ├── Master.cpp                    # Prueba 1 y 3
│   │   ├── Master_static.cpp             # Prueba 2
│   │   ├── Master_parking_bateria.cpp    # Prueba 4
│   │   └── Master_parking_paralelo.cpp   # Prueba 4
│   ├── CMakeLists.txt
│   └── package.xml
│
├── hardware_interface/            # Nodos de hardware en la Pi
│   ├── src/
│   │   ├── motor_driver.py        # Motores I2C + servo pigpio
│   │   ├── ultrasonic_sensors.py  # HC-SR04 x4 via pigpio
│   │   └── teleop_controller.py   # Joystick PS4
│   ├── CMakeLists.txt
│   └── package.xml
│
├── esp32_firmware/                # Firmware ESP32 (NO es paquete ROS 2)
│   └── avim_esp32_lights.ino      # Solo LEDs via micro-ROS
│
├── lane_detection/                # Detección de carril y señal STOP
│   ├── src/
│   │   ├── lane_detection.cpp     # Pipeline bird's-eye + sliding windows
│   │   └── stop_sign_detector.py
│   ├── CMakeLists.txt
│   └── package.xml
│
├── object_detection/              # Detección obstáculos + msg custom
│   ├── msg/
│   │   └── PointsObjects.msg
│   ├── src/
│   │   └── object_detection.cpp   # Adaptado para Nuwa-HP60C
│   ├── CMakeLists.txt
│   └── package.xml
│
└── object_detection_parking/      # Detección para estacionamiento
    ├── src/
    │   └── object_detection_parking.cpp  # Adaptado para Nuwa-HP60C
    ├── CMakeLists.txt
    └── package.xml
```

---

## Instalación rápida

> Para la guía completa paso a paso (instalación de OS, errores conocidos y soluciones,
> pruebas de cada componente, configuración de la Nuwa-HP60C, etc.) consulta
> **`AVIM_Setup_Guide_v3.docx`**.

```bash
# 1. Instalar ROS 2 Humble (ver guía completa)

# 2. Instalar dependencias del proyecto
sudo apt install -y \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-joy ros-humble-camera-ros \
  ros-humble-rqt-image-view \
  ros-humble-pcl-conversions ros-humble-pcl-ros ros-humble-pcl-msgs \
  libopencv-dev python3-opencv libpcl-dev i2c-tools \
  libgflags-dev nlohmann-json3-dev libgoogle-glog-dev

# Dependencias Python
pip3 install smbus2

# pigpio (compilar desde fuente — NO está en apt de Ubuntu 22.04)
cd ~ && git clone https://github.com/joan2937/pigpio.git
cd pigpio && make && sudo make install
cd ~

# Servicio pigpiod (ver guía completa)
sudo systemctl enable pigpiod && sudo systemctl start pigpiod

# 3. Compilar micro-ROS Agent desde fuente
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
source /opt/ros/humble/setup.bash
colcon build && source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
source install/local_setup.bash
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# 4. Descargar e instalar SDK de Nuwa-HP60C (workspace separado)
# El SDK se descarga desde:
# https://drive.google.com/drive/folders/1Vcm8kTs1em6Q1NqPe_3EopCoH3nbvHL4
# (ver guía completa para detalles)
cd ~/ascam_ros2_ws
chmod a+x build.sh
./build.sh

# Instalar librería SDK al sistema
sudo cp ~/ascam_ros2_ws/src/ascamera/libs/lib/aarch64-linux-gnu/*.so* /usr/local/lib/
sudo ldconfig

# Instalar reglas udev
cd ~/ascam_ros2_ws/src/ascamera/scripts
sudo bash create_udev_rules.sh

# 5. Crear workspace y clonar el proyecto
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/efralic/AutoModelCarROS2.git .

# 6. Compilar el proyecto
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --parallel-workers 1

# 7. Configurar .bashrc (orden importante)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/carrito/microros_ws/install/local_setup.bash" >> ~/.bashrc
echo "source /home/carrito/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source /home/carrito/ascam_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Uso

### Secuencia de arranque

```bash
# 1. Verificar que pigpiod está corriendo
sudo systemctl status pigpiod --no-pager

# 2. Verificar que el micro-ROS Agent está activo
sudo systemctl status microros-agent --no-pager

# 3. Verificar que la Nuwa-HP60C está conectada
lsusb | grep NOVATEK   # debe aparecer: 3482:6723

# 4. Lanzar la prueba deseada
ros2 launch avim tmr2026_main.launch.py prueba:=1
```

### Pruebas del TMR 2026

```bash
# Prueba 1 – Reconocimiento de señales STOP
ros2 launch avim tmr2026_main.launch.py prueba:=1

# Prueba 2 – Obstáculos estáticos
ros2 launch avim tmr2026_main.launch.py prueba:=2

# Prueba 3 – Obstáculos dinámicos
ros2 launch avim tmr2026_main.launch.py prueba:=3

# Prueba 4 – Estacionamiento
ros2 launch avim tmr2026_main.launch.py prueba:=4

# Solo teleoperación
ros2 launch avim teleop.launch.py
```

---

## Pruebas individuales

### Cámara Nuwa-HP60C (profundidad)

```bash
# Verificar que la cámara está conectada
lsusb | grep NOVATEK   # debe aparecer 3482:6723

# Lanzar el driver
ros2 launch ascamera hp60c.launch.py

# Listar tópicos
ros2 topic list | grep ascamera
# Debe aparecer:
#   /ascamera_hp60c/camera_publisher/depth0/camera_info
#   /ascamera_hp60c/camera_publisher/depth0/image_raw
#   /ascamera_hp60c/camera_publisher/depth0/points
#   /ascamera_hp60c/camera_publisher/rgb0/camera_info
#   /ascamera_hp60c/camera_publisher/rgb0/image

# Frecuencia de publicación (~25 FPS)
ros2 topic hz /ascamera_hp60c/camera_publisher/rgb0/image
ros2 topic hz /ascamera_hp60c/camera_publisher/depth0/image_raw

# Ver con VcXsrv
ros2 run rqt_image_view rqt_image_view
```

### Object detection (Nuwa-HP60C)

```bash
# Lanzar object_detection
ros2 run object_detection object_detection_node

# Ver obstáculos detectados
ros2 topic echo /objects_points
```

Pon un objeto a 50-100 cm de la cámara → debes ver mensajes con `dist=50-100 cm` y `angulo~90 deg`. Muévelo a izquierda → ángulo baja (~70°). Muévelo a derecha → ángulo sube (~110°).

### Motores (I2C desde la Pi)

```bash
# Verificar I2C
sudo i2cdetect -y 1   # debe aparecer "34"

# Lanzar nodo de motores
ros2 run hardware_interface motor_driver.py

# Adelante (negativo = adelante)
ros2 topic pub --once /AutoModelMini/manual_control/speed \
  std_msgs/msg/Int16 'data: -300'

# Reversa
ros2 topic pub --once /AutoModelMini/manual_control/speed \
  std_msgs/msg/Int16 'data: 300'

# Stop
ros2 topic pub --once /AutoModelMini/manual_control/speed \
  std_msgs/msg/Int16 'data: 0'
```

### Servo

```bash
# Centro (90°)
ros2 topic pub --once /AutoModelMini/manual_control/steering \
  std_msgs/msg/Int16 'data: 90'

# Izquierda máximo (65°)
ros2 topic pub --once /AutoModelMini/manual_control/steering \
  std_msgs/msg/Int16 'data: 65'

# Derecha máximo (115°)
ros2 topic pub --once /AutoModelMini/manual_control/steering \
  std_msgs/msg/Int16 'data: 115'
```

### Ultrasonidos

```bash
ros2 run hardware_interface ultrasonic_sensors.py

# En otra terminal
ros2 topic echo /sensors/ultrasonic/front
ros2 topic hz /sensors/ultrasonic/front   # ~10 Hz
```

### Arducam IMX219 (CSI)

```bash
ros2 run camera_ros camera_node --ros-args \
  -p width:=640 \
  -p height:=480 \
  -p format:=YUYV \
  -p camera:='/base/soc/i2c0mux/i2c@1/imx219@10'

# Frecuencia (~15 FPS)
ros2 topic hz /camera/image_raw
```

### Lane detection

```bash
# Requiere la Arducam corriendo primero
ros2 run lane_detection lane_detection --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p debug_output:=true

# Ver el error lateral
ros2 topic echo /distance_center_line
```

### LEDs ESP32 (micro-ROS)

```bash
# Requiere micro-ROS Agent corriendo y ESP32 conectado
ros2 node list | grep esp32   # debe aparecer /avim_esp32_lights

# Blinkers
ros2 topic pub --once /lights/blinkers std_msgs/msg/Bool 'data: true'

# Señales
ros2 topic pub --once /lights/left_signal std_msgs/msg/Bool 'data: true'
ros2 topic pub --once /lights/right_signal std_msgs/msg/Bool 'data: true'
```

---

## Tópicos ROS 2

### Publicados por los nodos

| Tópico | Tipo | Publicado por |
|---|---|---|
| `/distance_center_line` | `std_msgs/Int16` | `lane_detection` (Pi) |
| `/stop_sign_detected` | `std_msgs/Bool` | `stop_sign_detector` (Pi) |
| `/objects_points` | `object_detection/PointsObjects` | `object_detection` (Pi) |
| `/sensors/ultrasonic/front` | `sensor_msgs/Range` | `ultrasonic_sensors.py` |
| `/sensors/ultrasonic/back` | `sensor_msgs/Range` | `ultrasonic_sensors.py` |
| `/sensors/ultrasonic/left` | `sensor_msgs/Range` | `ultrasonic_sensors.py` |
| `/sensors/ultrasonic/right` | `sensor_msgs/Range` | `ultrasonic_sensors.py` |
| `/camera/image_raw` | `sensor_msgs/Image` | `camera_ros` (Arducam CSI) |
| `/ascamera_hp60c/camera_publisher/rgb0/image` | `sensor_msgs/Image` | `ascamera` (Nuwa color) |
| `/ascamera_hp60c/camera_publisher/depth0/image_raw` | `sensor_msgs/Image` | `ascamera` (Nuwa depth) |
| `/ascamera_hp60c/camera_publisher/depth0/points` | `sensor_msgs/PointCloud2` | `ascamera` (Nuwa points) |

### Suscritos / publicados por los Masters

| Tópico | Tipo | Dirección |
|---|---|---|
| `/AutoModelMini/manual_control/speed` | `std_msgs/Int16` | Master → `motor_driver.py` |
| `/AutoModelMini/manual_control/steering` | `std_msgs/Int16` | Master → `motor_driver.py` |
| `/lights/blinkers` | `std_msgs/Bool` | Master → ESP32 LEDs |
| `/lights/left_signal` | `std_msgs/Bool` | Master → ESP32 LEDs |
| `/lights/right_signal` | `std_msgs/Bool` | Master → ESP32 LEDs |
| `/vehicle_mode` | `std_msgs/String` | `teleop_controller.py` → `motor_driver.py` |
| `/emergency_stop` | `std_msgs/Bool` | `teleop_controller.py` → `motor_driver.py` |

---

## Dependencias

### ROS 2 (apt)

```
ros-humble-rclcpp         ros-humble-rclpy
ros-humble-std-msgs       ros-humble-sensor-msgs
ros-humble-geometry-msgs  ros-humble-cv-bridge
ros-humble-image-transport ros-humble-camera-ros
ros-humble-joy            ros-humble-rqt-image-view
ros-humble-pcl-conversions ros-humble-pcl-ros ros-humble-pcl-msgs
ros-humble-image-publisher
```

### Sistema

```
libopencv-dev   python3-opencv   python3-colcon-common-extensions
i2c-tools       libpcl-dev       libgflags-dev
nlohmann-json3-dev   libgoogle-glog-dev
smbus2 (pip3)
```

### pigpio (compilar desde fuente)

```bash
git clone https://github.com/joan2937/pigpio.git
cd pigpio && make && sudo make install
```

### ESP32 (Arduino IDE)

```
micro_ros_arduino
```

### micro-ROS Agent (compilar desde fuente)

```bash
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
```

### Nuwa-HP60C SDK (driver Angstrong)

```
Workspace: ~/ascam_ros2_ws (separado del proyecto)
SDK descargado de: https://drive.google.com/drive/folders/1Vcm8kTs1em6Q1NqPe_3EopCoH3nbvHL4
Tutoriales: https://github.com/YahboomTechnology/Nuwa-HP60C-Depth-Camera
```

---

## Controles PS4

| Botón/Eje | Acción |
|---|---|
| R2 | Acelerar hacia adelante |
| L2 | Reversa |
| Stick izquierdo (horizontal) | Dirección |
| Cruz ✕ | Cambiar modo Autónomo ↔ Teleop |
| Triángulo △ | Paro de emergencia |

---

## Notas importantes

### Cámara Arducam IMX219 (CSI)

- Usar **`camera_ros`**, NO `v4l2_camera` (no funciona con CSI en Ubuntu 22.04)
- Requiere overlay en `/boot/firmware/config.txt`:
  ```ini
  camera_auto_detect=0
  dtoverlay=imx219
  ```
- El overlay debe estar en la sección `[all]`, NO en `[arm64]`
- Publica en `/camera/image_raw` a ~15 FPS con formato NV21

### Cámara Nuwa-HP60C

- El SDK requiere un **workspace separado** (`~/ascam_ros2_ws`) para no contaminar el proyecto principal
- La librería `libAngstrongCameraSdk.so` se debe instalar en `/usr/local/lib/` con `sudo ldconfig`
- Las reglas udev se instalan con `sudo bash create_udev_rules.sh`
- Editar `hp60c.launch.py` para cambiar `/home/yahboom/` → `/home/<tu_usuario>/`
- Publica color a ~25 FPS, depth en formato `16UC1` (mm) a la misma frecuencia

### Orden del .bashrc (crítico)

```bash
source /opt/ros/humble/setup.bash                               # 1° siempre
source /home/carrito/microros_ws/install/local_setup.bash       # 2°
source /home/carrito/ros2_ws/install/setup.bash                 # 3°
source /home/carrito/ascam_ros2_ws/install/setup.bash           # 4°
```

### VcXsrv para visualización X11

- Usar **VcXsrv** en lugar de Xming (Xming no soporta GLX moderno)
- Configurar con "Native opengl" y "Disable access control" marcados
- En PuTTY, dejar el campo "X display location" **vacío** (no `localhost:0.0`)

---

## Equipo

> TMR 2026 — AVIM Team

---

## Licencia

MIT License — ver `LICENSE` para detalles.
