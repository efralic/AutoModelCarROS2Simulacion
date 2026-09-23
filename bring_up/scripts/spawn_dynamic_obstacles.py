#!/usr/bin/env python3
"""
Escenario "obstáculos dinámicos" (prueba 3 del TMR).
Puerto a ROS 2 de test3_start.py.

Cambios frente a la versión ROS 1:
  - /gazebo/spawn_sdf_model (SpawnModel) -> /spawn_entity (SpawnEntity)
  - /gazebo/get_link_state (GetLinkState) -> /get_entity_state
    (GetEntityState). ROS 2 (libgazebo_ros_state.so) ya no expone un
    servicio por-link; se consulta el estado del modelo completo
    "AutoModel_Obstacle1" en vez de "AutoModel_Obstacle1::base_link".
    Verifica con `ros2 service list` que el nombre te sirva tal cual;
    si tu modelo tuviera varios links relevantes, se puede pedir el
    estado de un link específico igual con el nombre con "::".

Spawnea:
  - AutoModelMini        (AutoNOMOS_mini)
  - StopSign             (stop_sign_small)
  - AutoModel_Obstacle1  (AutoNOMOS_mini_obstacle)  <- el que se mueve
  - AutoModel_Obstacle2/3 (AutoNOMOS_mini_static)

Y controla a AutoModel_Obstacle1 en un lazo, igual que en la versión ROS 1.
"""
import math
import os
import random
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, GetEntityState
from std_msgs.msg import Int16
from ament_index_python.packages import get_package_share_directory


def spawn(node, client, pkg_models_dir, name, model_dir, x, y, z,
          yaw=None, qz=None, qw=None):
    req = SpawnEntity.Request()
    req.name = name
    with open(os.path.join(pkg_models_dir, model_dir, 'model.sdf')) as f:
        req.xml = f.read()
    req.robot_namespace = ''
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = z
    if yaw is not None:
        req.initial_pose.orientation.z = math.sin(yaw / 2.0)
        req.initial_pose.orientation.w = math.cos(yaw / 2.0)
    else:
        req.initial_pose.orientation.z = qz
        req.initial_pose.orientation.w = qw

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result is None or not result.success:
        node.get_logger().warn(f'No se pudo spawnear {name}: {result}')
    else:
        node.get_logger().info(f'Spawneado: {name}')


def main():
    rclpy.init()
    node = Node('spawn_dynamic_obstacles')

    spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
    node.get_logger().info('Esperando /spawn_entity...')
    spawn_client.wait_for_service()

    pkg_models_dir = os.path.join(
        get_package_share_directory('autonomos_gazebo_simulation'), 'models')

    angle = random.uniform(math.pi - 0.3, math.pi + 0.3)
    pos_x = random.uniform(1.2, 3.8)
    spawn(node, spawn_client, pkg_models_dir, 'AutoModelMini',
          'AutoNOMOS_mini', pos_x, 3.3, 0.17, yaw=angle)

    spawn(node, spawn_client, pkg_models_dir, 'StopSign',
          'stop_sign_small', pos_x - 0.2, 3.55, 0.0, yaw=1.5708)

    spawn(node, spawn_client, pkg_models_dir, 'AutoModel_Obstacle1',
          'AutoNOMOS_mini_obstacle', pos_x - 2, 3.3, 0.17, qz=1.0, qw=0.0)

    angle2 = random.uniform(0, 3.0 * math.pi / 2.0)
    pos_x2 = -2.2 + 1.55 * math.cos(angle2)
    pos_y2 = 0.5 + 1.55 * math.sin(angle2)
    spawn(node, spawn_client, pkg_models_dir, 'AutoModel_Obstacle2',
          'AutoNOMOS_mini_static', pos_x2, pos_y2, 0.17,
          yaw=angle2 + math.pi / 2)

    angle3 = random.uniform(-math.pi / 2.0 - 0.2, -math.pi / 2.0 + 0.2)
    pos_y3 = random.uniform(-1.25, -1.5)
    spawn(node, spawn_client, pkg_models_dir, 'AutoModel_Obstacle3',
          'AutoNOMOS_mini_static', 0.65, pos_y3, 0.17, yaw=angle3)

    # ---- Lazo de control del obstáculo dinámico (AutoModel_Obstacle1) ----
    get_state_client = node.create_client(GetEntityState, '/get_entity_state')
    node.get_logger().info('Esperando /get_entity_state...')
    get_state_client.wait_for_service()

    pub_speed = node.create_publisher(
        Int16, '/AutoModel_Obstacle1/manual_control/speed', 1)
    pub_steer = node.create_publisher(
        Int16, '/AutoModel_Obstacle1/manual_control/steering', 1)

    initial_speed = -random.randint(80, 120)

    try:
        while rclpy.ok():
            req = GetEntityState.Request()
            req.name = 'AutoModel_Obstacle1'
            future = get_state_client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            result = future.result()

            if result is None or not result.success:
                time.sleep(0.1)
                continue

            obstacle_x = result.state.pose.position.x
            obstacle_y = result.state.pose.position.y

            steering = 135 if obstacle_x < -4.18 else 90 - int(100 * (3.3 - obstacle_y))
            speed = initial_speed if obstacle_y > 1.8 else 0

            pub_speed.publish(Int16(data=speed))
            pub_steer.publish(Int16(data=steering))

            time.sleep(0.1)  # 10 Hz, igual que la versión ROS 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
