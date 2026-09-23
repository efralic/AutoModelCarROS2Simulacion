#!/usr/bin/env python3
"""
Escenario "estacionamiento" (prueba 4 del TMR).
Puerto a ROS 2 de test4_start.py.

Spawnea:
  - AutoModelMini                     (AutoNOMOS_mini)
  - AutoModelParked1/2/3/4            (AutoNOMOS_mini_static), posiciones fijas
"""
import math
import os
import random

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
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
    node = Node('spawn_parking')

    client = node.create_client(SpawnEntity, '/spawn_entity')
    node.get_logger().info('Esperando /spawn_entity...')
    client.wait_for_service()

    pkg_models_dir = os.path.join(
        get_package_share_directory('autonomos_gazebo_simulation'), 'models')

    angle = random.uniform(math.pi - 0.3, math.pi + 0.3)
    pos_x = random.uniform(0.4, 0.6)
    spawn(node, client, pkg_models_dir, 'AutoModelMini', 'AutoNOMOS_mini',
          pos_x, 3.3, 0.17, yaw=angle)

    posiciones = [(-1.0, 3.64), (-1.5, 3.64), (-2.88, 3.64), (-3.38, 3.64)]
    for i, (x, y) in enumerate(posiciones, start=1):
        spawn(node, client, pkg_models_dir, f'AutoModelParked{i}',
              'AutoNOMOS_mini_static', x, y, 0.17, qz=-1.0, qw=0.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
