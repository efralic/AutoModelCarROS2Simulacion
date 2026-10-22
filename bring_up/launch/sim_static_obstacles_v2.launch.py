"""
sim_static_obstacles_v2.launch.py
════════════════════════════════════════════════════════════════
Simulación TMR — Prueba 2 (obstáculos ESTÁTICOS) con el Master NUEVO:
Master_overtake (rebase en lazo cerrado: carril + cámara de profundidad +
4 ultrasónicos + odometría).

Diferencias con sim_static_obstacles.launch.py:
  • Master_overtake en lugar de Master_static
  • use_sim_time:=true en TODOS los nodos (los tiempos son de Gazebo)
  • lane_detection publica la geometría del carril (/lane/left_line,
    /lane/right_line) que usa el Master nuevo

IMPORTANTE: copia en LANE_PARAMS y OBJ_PARAMS los parámetros que ya
calibraste en tu sim_static_obstacles.launch.py (trapecio, smoothing_frames,
horizon_row / camera_height_m del filtro de suelo, etc.).

Uso:
  ros2 launch bring_up sim_static_obstacles_v2.launch.py
  ros2 launch bring_up sim_static_obstacles_v2.launch.py debug:=true
════════════════════════════════════════════════════════════════
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ── Tus parámetros calibrados (cópialos de tu launch actual) ─────────────────
LANE_PARAMS = {
    'camera_topic': '/camera/image_raw',
    'smoothing_frames': 3,
    'src_tl_x': 70, 
    'src_tr_x': 570, 
    'src_top_y': 240,          # <- tu trapecio
    'src_bl_x': 0, 
    'src_br_x': 639,
    'src_bot_y': 470,  
    # ...
    # Geometría del carril para el Master nuevo (cámara simulada):
    'publish_lane_geometry': True,
    # No perder las líneas cuando el carro va inclinado (cambio de carril).
    # 'maneuver' = solo mientras rebasa (en el carril normal se comporta igual
    # que tu detector original, con el que tomas bien las curvas)
    # 'always' / 'off' (también acepta True / False)
    'robust_base': 'maneuver',
    'hist_window': 31,
    'hist_min_px': 150,
    'camera_hfov_deg': 73.8,
    'horizon_row': 214.5,
    'camera_height_m': 0.165,
}

OBJ_PARAMS = {
    'depth_topic': '/camera/depth/image_raw',
    'horizon_row': 224.6,      # <- los de calibrar_suelo.py
    'camera_height_m': 0.1331,
    'range_max_mm': 2500.0,
    'v_end_ratio': 0.78,
    'v_start_ratio': 0.0,
    'ground_filter': True,
}

MASTER_PARAMS = {
    # 0 = tu on_lane (con /distance_center_line), 1 = pure pursuit con la geometría
    'lane_keep_mode': 0,
    # True = usar el giroscopio /imu si existe. False = solo odometría por
    # comandos (como en el carro real sin IMU); entonces conviene maniobrar
    # más despacio:
    'use_imu': False,
    # 'change_cmd': 200,
    # 'pass_cmd': 240,
    # Simulación: estos son los valores del plugin de Gazebo. En el carro real
    # hay que MEDIRLOS (ver guía).
    'speed_mps_per_cmd': 0.00122,
    'steer_deg_per_cmd': 0.1689,
    'steer_offset_deg': 0.314,
    'max_steer_deg': 14.9,
    'auto_steer_gain': False,
    'auto_speed_scale': False,
    # ── maniobras más largas: con ~15° de rueda no se puede cambiar de carril en 1.2 m ──
    'change_length_m': 1.6,     # antes 1.2
    'return_length_m': 1.9,     # antes 1.6
    'trigger_gap_m': 1.1,       # antes 0.85: empieza a esquivar antes
    'slow_gap_m': 1.7,          # antes 1.4
    # ── tu on_lane: más rango para las curvas ──
    'lk_min_cmd': 45.0,         # antes 30
    'lk_max_cmd': 150.0,        # antes 135
    # Alcance real de object_detection (range_max_mm) menos un margen
    'depth_range_m': 2.4,
}

STOP_PARAMS = {
    'camera_topic': '/camera/image_raw',
    'depth_topic': '/camera/depth/image_raw',   # distancia por profundidad ('' = por tamaño)
    'camera_hfov_deg': 73.8,
    'sign_size_m': 0.139,      # alto del octágono rojo de stop_sign_tmr
    'side': 'right',           # solo señales a la derecha del carril
    'min_lateral_m': -0.15,
    'max_lateral_m': 0.65,
    'max_distance_m': 2.5,
    'detection_threshold': 3,
}

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
        parameters=[LANE_PARAMS, sim],
        output='screen',
    )

    object_detection_node = Node(
        package='object_detection',
        executable='object_detection_node',
        name='object_detection',
        parameters=[OBJ_PARAMS, sim],
        output='screen',
    )

    stop_sign_node = Node(
        package='lane_detection',
        executable='stop_sign_detector.py',
        name='stop_sign_detector',
        parameters=[STOP_PARAMS, sim],
        output='screen',
    )

    # El Master arranca cuando ya hay carro y obstáculos en Gazebo
    master_node = TimerAction(
        period=8.0,
        actions=[Node(
            package='control',
            executable='Master_overtake',
            name='Master_overtake',
            parameters=[MASTER_PARAMS, sim, {'debug': debug}],
            output='screen',
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false',
                              description='Imprime estado y sensores 2 veces por segundo'),
        gazebo,
        spawn_robot,
        spawn_obstacles,
        lane_detection_node,
        object_detection_node,
	stop_sign_node,
        master_node,
    ])
