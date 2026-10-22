"""
sim_no_obstacles.launch.py
════════════════════════════════════════════════════════════════
Simulación TMR — Prueba 1: navegación SIN obstáculos (señales de tránsito)

Lanza: Gazebo (mundo TMR2021) + AutoNOMOS_mini + StopSign (spawneados por
spawn_no_obstacles.py) + lane_detection + stop_sign_detector + Master.

El Master se detiene EN EL CRUCE (defensa stop_before_m antes de la señal),
no en cuanto ve la señal: usa /stop_sign_position del detector v2.

No incluye motor_driver.py / ultrasonic_sensors.py: esos son solo para la
Raspberry Pi real, aquí el movimiento lo da el plugin de Gazebo
(gazebo_plugin) al leer /AutoModelMini/manual_control/*.

IMPORTANTE: confirma con `ros2 topic list` una vez levantada la sim que la
cámara simulada publica en efecto bajo /camera/image_raw. Si el nombre
resultara distinto, ajusta camera_topic abajo (no hace falta tocar el
código de lane_detection/stop_sign_detector: ya lo reciben como parámetro).
════════════════════════════════════════════════════════════════
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('autonomos_gazebo_simulation'),
                'worlds', 'TMR2021.world'
            ]),
            'verbose': 'true',
        }.items(),
    )

    # Da tiempo a que gzserver termine de arrancar antes de pedir el spawn.
    spawn_scene = TimerAction(
        period=3.0,
        actions=[Node(
            package='bring_up',
            executable='spawn_no_obstacles.py',
            name='spawn_no_obstacles',
            output='screen',
        )],
    )

    lane_detection_node = Node(
        package='lane_detection',
        executable='lane_detection',
        name='lane_detection',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'debug_output': True,
            'smoothing_frames': 3,
        }],
        output='screen',
    )

    stop_sign_node = Node(
        package='lane_detection',
        executable='stop_sign_detector.py',
        name='stop_sign_detector',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',  # distancia por profundidad ('' = por tamaño)
            'camera_hfov_deg': 73.8,
            'sign_size_m': 0.139,      # alto del octágono rojo de stop_sign_tmr
            'side': 'right',           # solo señales a la derecha del carril
            'min_lateral_m': -0.15,
            'max_lateral_m': 0.65,
            'max_distance_m': 2.5,
            'detection_threshold': 3,
            'use_sim_time': True,
        }],
        output='screen',
    )

    master_node = Node(
        package='control',
        executable='Master',
        name='Master',
        parameters=[{
            'use_sim_time': True,          # tiempos de Gazebo (odometría y los 2 s del alto)
            'speed_mps_per_cmd': 0.00122,  # de calibrar_odometria.py
            'stop_before_m': 0.06,         # defensa 6 cm antes de la señal (11 cm antes de la cebra)
            'stop_time_s': 2.0,
        }],
        output='screen',
    )


    return LaunchDescription([
        gazebo,
        spawn_scene,
        lane_detection_node,
        stop_sign_node,
        master_node,
    ])
