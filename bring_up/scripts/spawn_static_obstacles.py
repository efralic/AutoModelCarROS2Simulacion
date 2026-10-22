#!/usr/bin/env python3
"""
Escenario "obstáculos estáticos" (prueba 2 del TMR).
Puerto a ROS 2 de test2_start.py. El robot principal (AutoModelMini) ya se
spawnea aparte en sim_static_obstacles.launch.py; este script solo agrega
los 3 obstáculos estáticos.

Spawnea:
  - AutoModel_Obstacle2/3/4  (AutoNOMOS_mini_static), posiciones aleatorias
"""
import math
import os
import random

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


def spawn(node, client, pkg_models_dir, name, model_dir, x, y, z, yaw):
    req = SpawnEntity.Request()
    req.name = name
    with open(os.path.join(pkg_models_dir, model_dir, 'model.sdf')) as f:
        req.xml = f.read()
    req.robot_namespace = ''
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = z
    req.initial_pose.orientation.z = math.sin(yaw / 2.0)
    req.initial_pose.orientation.w = math.cos(yaw / 2.0)

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result is None or not result.success:
        node.get_logger().warn(f'No se pudo spawnear {name}: {result}')
    else:
        node.get_logger().info(f'Spawneado: {name}')


def main():
    rclpy.init()
    node = Node('spawn_static_obstacles')

    client = node.create_client(SpawnEntity, '/spawn_entity')
    node.get_logger().info('Esperando /spawn_entity...')
    client.wait_for_service()

    pkg_models_dir = os.path.join(
        get_package_share_directory('autonomos_gazebo_simulation'), 'models')

    angle = random.uniform(-math.pi / 2.0 - 0.2, -math.pi / 2.0 + 0.2)
    pos_y = random.uniform(-1, 1)
    spawn(node, client, pkg_models_dir, 'AutoModel_Obstacle2',
          'AutoNOMOS_mini_static', -5.4, pos_y, 0.17, angle)

    angle = random.uniform(0, 3.0 * math.pi / 2.0)
    pos_x = -2.2 + 1.55 * math.cos(angle)
    pos_y = 0.5 + 1.55 * math.sin(angle)
    spawn(node, client, pkg_models_dir, 'AutoModel_Obstacle3',
          'AutoNOMOS_mini_static', pos_x, pos_y, 0.17, angle + math.pi / 2)

    angle = random.uniform(-math.pi / 2.0 - 0.2, -math.pi / 2.0 + 0.2)
    pos_y = random.uniform(-1.25, -1.5)
    spawn(node, client, pkg_models_dir, 'AutoModel_Obstacle4',
          'AutoNOMOS_mini_static', 0.65, pos_y, 0.17, angle)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
