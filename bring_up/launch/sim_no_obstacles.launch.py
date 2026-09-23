"""
sim_no_obstacles.launch.py
════════════════════════════════════════════════════════════════
Simulación TMR — Prueba 1: navegación SIN obstáculos (señales de tránsito)

Lanza: Gazebo (mundo TMR2021) + AutoNOMOS_mini + StopSign (spawneados por
spawn_no_obstacles.py) + lane_detection + stop_sign_detector + Master.

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
        parameters=[{'camera_topic': '/camera/image_raw'}],
        output='screen',
    )

    stop_sign_node = Node(
        package='lane_detection',
        executable='stop_sign_detector.py',
        name='stop_sign_detector',
        parameters=[{'camera_topic': '/camera/image_raw'}],
        output='screen',
    )

    master_node = Node(
        package='control',
        executable='Master',
        name='Master',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        spawn_scene,
        lane_detection_node,
        stop_sign_node,
        master_node,
    ])
