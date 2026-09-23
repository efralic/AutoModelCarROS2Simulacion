"""
sim_static_obstacles.launch.py
════════════════════════════════════════════════════════════════
Simulación TMR — Prueba 2: navegación con obstáculos ESTÁTICOS

Lanza: Gazebo (mundo TMR2021) + AutoNOMOS_mini (spawn_entity.py) +
3 obstáculos estáticos (spawn_static_obstacles.py) + lane_detection +
object_detection (cámara de profundidad simulada, ya NO LIDAR) +
Master_static.
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
        }.items(),
    )

    model_sdf = PathJoinSubstitution([
        FindPackageShare('autonomos_gazebo_simulation'),
        'models', 'AutoNOMOS_mini', 'model.sdf'
    ])

    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_auto_model_mini',
            arguments=['-entity', 'AutoModelMini', '-file', model_sdf,
                       '-x', '2.5', '-y', '3.3', '-z', '0.17', '-Y', '3.14159'],
            output='screen',
        )],
    )

    spawn_obstacles = TimerAction(
        period=5.0,
        actions=[Node(
            package='bring_up',
            executable='spawn_static_obstacles.py',
            name='spawn_static_obstacles',
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

    object_detection_node = Node(
        package='object_detection',
        executable='object_detection_node',
        name='object_detection',
        parameters=[{'depth_topic': '/camera/depth/image_raw'}],
        output='screen',
    )

    master_static_node = Node(
        package='control',
        executable='Master_static',
        name='Master_static',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        spawn_obstacles,
        lane_detection_node,
        object_detection_node,
        master_static_node,
    ])
