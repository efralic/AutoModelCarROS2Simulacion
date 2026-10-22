"""
sim_parking_v2.launch.py
════════════════════════════════════════════════════════════════
Simulación TMR — Estacionamiento en PARALELO con Master_parking_v2.

  • Mundo TMR2021_parking.world (pista + pared) y spawn_parking.py (tu
    carro + 4 carros estacionados, con un hueco entre el 2º y el 3º).
  • lane_detection para seguir el carril mientras busca el lugar.
  • Master_parking_v2: busca el hueco con el ultrasónico derecho, se
    estaciona en reversa (derecha → recto → izquierda), se endereza si hace
    falta y se centra con los ultrasónicos delantero y trasero. Si quedó
    lejos de la pared, se corrige con el ultrasónico derecho (sin IMU).

Los parámetros de lane_detection y de la calibración del carro
(steer_deg_per_cmd, speed_mps_per_cmd, lk_kp_*, use_imu, …) se toman de
TU sim_static_obstacles_v2.launch.py.

Uso:
  ros2 launch bring_up sim_parking_v2.launch.py
  ros2 launch bring_up sim_parking_v2.launch.py debug:=true
  ros2 launch bring_up sim_parking_v2.launch.py run_master:=false   (para calibrar_giro.py)
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


def _static_params():
    here = os.path.dirname(os.path.realpath(__file__))
    for d in (here, os.path.dirname(os.path.abspath(__file__))):
        path = os.path.join(d, 'sim_static_obstacles_v2.launch.py')
        if os.path.exists(path):
            spec = importlib.util.spec_from_file_location('sim_static_obstacles_v2', path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    return None


S = _static_params()
LANE_PARAMS = dict(getattr(S, 'LANE_PARAMS', {'camera_topic': '/camera/image_raw'})) if S else {'camera_topic': '/camera/image_raw'}
_M = dict(getattr(S, 'MASTER_PARAMS', {})) if S else {}

# De tu MASTER_PARAMS solo se usan los que también entiende Master_parking_v2
_SHARED = ('steer_deg_per_cmd', 'steer_offset_deg', 'max_steer_deg', 'speed_mps_per_cmd',
           'use_imu', 'steer_lag_s', 'lk_min_cmd', 'lk_max_cmd',
           'lk_thr_0', 'lk_thr_1', 'lk_thr_2', 'lk_thr_3',
           'lk_kp_0', 'lk_kp_1', 'lk_kp_2', 'lk_kp_3', 'lk_kp_4')
PARK_PARAMS = {k: v for k, v in _M.items() if k in _SHARED}
PARK_PARAMS.update({
    # Radio de giro mínimo (dirección al tope), MEDIDO con calibrar_giro.py.
    # IMPORTANTE sin IMU: el rumbo se calcula con este radio; si no es el real,
    # la maniobra no cuadra (0 = calcularlo con max_steer_deg).
    'turn_radius_m': 0.717,
    # Cómo responde la dirección. En Gazebo el plugin la mueve con un PID lento:
    # rodando tarda ~1.5 s en llegar al tope y detenido casi no gira. Estos
    # valores salen de tu log; MÍDELOS con calibrar_direccion.py y cámbialos.
    # (Carro real con servo: steer_lag_s 0.15, steer_static_frac 1.0)
    'steer_lag_s': 0.43,
    'steer_static_frac': 0.45,
    'steer_right_cmd': 0.0,    # dirección toda a la derecha
    'steer_left_cmd': 180.0,   # toda a la izquierda
    # Velocidades (comando; negativo = adelante en el plugin)
    'search_cmd': 250,         # buscando el lugar
    'maneuver_cmd': 150,       # en la maniobra
    # Lugar
    'min_spot_m': 0.62,        # hueco mínimo que acepta (el carro mide 0.43 m)
    'start_past_m': 0.30,      # eje trasero adelante de la defensa del carro de enfrente al empezar
    'arc_deg': 28.0,           # giro del primer arco
    # Seguridad
    'back_stop_m': 0.10,
    'front_stop_m': 0.10,
    # Profundidad (qué tan metido hacia la pared queda), funciona SIN IMU:
    # al final compara el ultrasónico derecho con la pared medida al pasar
    # junto al hueco y, si quedó unos cm lejos, hace una "S" corta.
    'max_depth_fixes': 3,          # 0 = desactivar la corrección
    'depth_tol_m': 0.02,           # corrige si quedó más de 2 cm lejos de lo ideal
    'final_wall_margin_m': 0.04,   # holgura final mínima con la pared
    'depth_bias_m': 0.0,           # + = más metido (p. ej. 0.01 = 1 cm más)
    'speed_lag_s': 0.24,           # retardo de arranque/frenado (calibrar_direccion.py)
    'depth_max_m': 0.10,           # si quedó más lejos que esto, la S no alcanza: solo avisa
})


def generate_launch_description():
    sim = {'use_sim_time': True}

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
            package='bring_up', executable='spawn_parking.py', name='spawn_parking', output='screen',
        )],
    )

    lane_detection_node = Node(
        package='lane_detection', executable='lane_detection', name='lane_detection',
        parameters=[LANE_PARAMS, sim], output='screen',
    )

    master_node = TimerAction(
        period=8.0,
        condition=IfCondition(LaunchConfiguration('run_master')),
        actions=[Node(
            package='control', executable='Master_parking_v2', name='Master_parking_v2',
            parameters=[PARK_PARAMS, sim, {'debug': LaunchConfiguration('debug')}],
            output='screen',
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false',
                              description='Imprime estado y sensores 2 veces por segundo'),
        DeclareLaunchArgument('run_master', default_value='true',
                              description='false = no lanzar el Master (p. ej. para calibrar_giro.py)'),
        gazebo,
        spawn_scene,
        lane_detection_node,
        master_node,
    ])
