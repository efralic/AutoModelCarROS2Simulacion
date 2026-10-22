"""
sim_dynamic_obstacles_v2.launch.py
════════════════════════════════════════════════════════════════
Simulación TMR — un obstáculo EN MOVIMIENTO con el Master_overtake.

  • Mismo mundo (TMR2021.world, con las señales de alto). No hace falta
    otro mundo: el obstáculo se agrega con spawn_moving_obstacle.py.
  • UN solo carrito obstáculo, en la recta más larga (la de arriba,
    donde arranca el carro), yendo despacio (speed_mps) por el carril
    derecho. Se detiene antes de la curva y, cuando el carro ya no está
    en esa recta, regresa al inicio para la siguiente vuelta.
  • Mismo Master_overtake y misma lógica de rebase que con obstáculos
    estáticos (rebasa con el ultrasónico derecho; funciona bien hasta
    ~0.15 m/s del obstáculo).

Los parámetros (LANE_PARAMS, OBJ_PARAMS, MASTER_PARAMS, STOP_PARAMS) se
toman de TU sim_static_obstacles_v2.launch.py, así que todo lo que ya
calibraste se usa igual aquí.

Uso:
  ros2 launch bring_up sim_dynamic_obstacles_v2.launch.py
  ros2 launch bring_up sim_dynamic_obstacles_v2.launch.py debug:=true
  ros2 launch bring_up sim_dynamic_obstacles_v2.launch.py obstacle_speed:=0.08
════════════════════════════════════════════════════════════════
"""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _static_params():
    """Lee los diccionarios de parámetros de sim_static_obstacles_v2.launch.py."""
    here = os.path.dirname(os.path.realpath(__file__))
    for d in (here, os.path.dirname(os.path.abspath(__file__))):
        path = os.path.join(d, 'sim_static_obstacles_v2.launch.py')
        if os.path.exists(path):
            spec = importlib.util.spec_from_file_location('sim_static_obstacles_v2', path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    raise RuntimeError('No encuentro sim_static_obstacles_v2.launch.py junto a este launch')


S = _static_params()
LANE_PARAMS = dict(getattr(S, 'LANE_PARAMS', {}))
OBJ_PARAMS = dict(getattr(S, 'OBJ_PARAMS', {}))
MASTER_PARAMS = dict(getattr(S, 'MASTER_PARAMS', {}))
STOP_PARAMS = dict(getattr(S, 'STOP_PARAMS', {
    'camera_topic': '/camera/image_raw',
    'depth_topic': '/camera/depth/image_raw',
    'camera_hfov_deg': 73.8,
    'sign_size_m': 0.139,
    'side': 'right',
    'min_lateral_m': -0.15,
    'max_lateral_m': 0.65,
    'max_distance_m': 2.5,
    'detection_threshold': 3,
}))

# Ajustes SOLO para el obstáculo en movimiento (descomenta si los necesitas):
# MASTER_PARAMS['side_clear_extra_m'] = 0.25   # avanzar más antes de regresar (el obstáculo también avanza)


def generate_launch_description():
    debug = LaunchConfiguration('debug')
    sim = {'use_sim_time': True}

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

    # El carro del Master: en la recta de arriba, hacia el oeste
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

    # El obstáculo en movimiento (lo spawnea y lo maneja)
    moving_obstacle = TimerAction(
        period=6.0,
        actions=[Node(
            package='bring_up',
            executable='spawn_moving_obstacle.py',
            name='spawn_moving_obstacle',
            parameters=[{
                'speed_mps': ParameterValue(LaunchConfiguration('obstacle_speed'), value_type=float),
                'start_x': 0.5,     # 2 m adelante del carro
                'end_x': -3.6,      # se detiene antes de la curva
                'lane_y': 3.28,
                'respawn': True,
            }],
            output='screen',
        )],
    )

    lane_detection_node = Node(
        package='lane_detection', executable='lane_detection', name='lane_detection',
        parameters=[LANE_PARAMS, sim], output='screen',
    )
    object_detection_node = Node(
        package='object_detection', executable='object_detection_node', name='object_detection',
        parameters=[OBJ_PARAMS, sim], output='screen',
    )
    stop_sign_node = Node(
        package='lane_detection', executable='stop_sign_detector.py', name='stop_sign_detector',
        condition=IfCondition(LaunchConfiguration('stop_signs')),
        parameters=[STOP_PARAMS, sim], output='screen',
    )

    # El Master arranca cuando ya están el carro y el obstáculo
    master_node = TimerAction(
        period=9.0,
        condition=IfCondition(LaunchConfiguration('run_master')),
        actions=[Node(
            package='control', executable='Master_overtake', name='Master_overtake',
            parameters=[MASTER_PARAMS, sim, {'debug': debug}], output='screen',
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false',
                              description='Imprime estado y sensores 2 veces por segundo'),
        DeclareLaunchArgument('run_master', default_value='true',
                              description='false = no lanzar el Master'),
        DeclareLaunchArgument('stop_signs', default_value='true',
                              description='false = no lanzar el detector de señales de alto'),
        DeclareLaunchArgument('obstacle_speed', default_value='0.12',
                              description='Velocidad del obstáculo en m/s (recomendado 0.08–0.15)'),
        gazebo,
        spawn_robot,
        moving_obstacle,
        lane_detection_node,
        object_detection_node,
        stop_sign_node,
        master_node,
    ])
