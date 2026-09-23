"""
sim_dynamic_obstacles.launch.py
════════════════════════════════════════════════════════════════
Simulación TMR — Prueba 3: navegación con obstáculos DINÁMICOS

Lanza: Gazebo (mundo TMR2021, con libgazebo_ros_state.so ya habilitado) +
escena completa (spawn_dynamic_obstacles.py hace el spawn Y controla al
obstáculo móvil) + lane_detection + object_detection + Master.
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

    spawn_scene = TimerAction(
        period=3.0,
        actions=[Node(
            package='bring_up',
            executable='spawn_dynamic_obstacles.py',
            name='spawn_dynamic_obstacles',
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
        object_detection_node,
        master_node,
    ])
