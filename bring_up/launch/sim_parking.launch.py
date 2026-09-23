"""
sim_parking.launch.py
════════════════════════════════════════════════════════════════
Simulación TMR — Prueba 4: estacionamiento autónomo

Lanza: Gazebo (mundo TMR2021_parking) + AutoNOMOS_mini + 4 autos
estacionados (spawn_parking.py) + lane_detection + object_detection_parking
+ Master_parking_paralelo.

Nota: el launch original (ROS 1) usaba "Master_parking_paralelo"; tu
AutoModelCarROS2 también tiene "Master_parking_bateria" para el modo de
batería. Cambia el executable de master_parking_node si quieres probar
ese modo en su lugar.
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
                'worlds', 'TMR2021_parking.world'
            ]),
        }.items(),
    )

    spawn_scene = TimerAction(
        period=3.0,
        actions=[Node(
            package='bring_up',
            executable='spawn_parking.py',
            name='spawn_parking',
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

    object_detection_parking_node = Node(
        package='object_detection_parking',
        executable='object_detection_parking',
        name='object_detection_parking',
        parameters=[{'depth_topic': '/camera/depth/image_raw'}],
        output='screen',
    )

    master_parking_node = Node(
        package='control',
        executable='Master_parking_paralelo',
        name='Master_parking_paralelo',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        spawn_scene,
        lane_detection_node,
        object_detection_parking_node,
        master_parking_node,
    ])
