#!/usr/bin/env python3
"""
Escenario "sin obstáculos" (prueba de señales de tránsito).
Puerto a ROS 2 de test1_start.py (rospy -> rclpy, servicio
/gazebo/spawn_sdf_model -> /spawn_entity).

Spawnea:
  - AutoModelMini  (AutoNOMOS_mini)
  - StopSign       (stop_sign_small)
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
    node = Node('spawn_no_obstacles')

    client = node.create_client(SpawnEntity, '/spawn_entity')
    node.get_logger().info('Esperando /spawn_entity...')
    client.wait_for_service()

    pkg_models_dir = os.path.join(
        get_package_share_directory('autonomos_gazebo_simulation'), 'models')

    angle = random.uniform(math.pi - 0.3, math.pi + 0.3)
    pos_x = random.uniform(1.2, 3.8)

    spawn(node, client, pkg_models_dir, 'AutoModelMini', 'AutoNOMOS_mini',
          pos_x, 3.3, 0.17, angle)
    spawn(node, client, pkg_models_dir, 'StopSign', 'stop_sign_small',
          pos_x - 0.2, 3.55, 0.0, 1.5708)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
