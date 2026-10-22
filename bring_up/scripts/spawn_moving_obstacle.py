#!/usr/bin/env python3
"""
spawn_moving_obstacle.py — UN solo obstáculo en movimiento en la recta más
larga de la pista (la de arriba, y ≈ 3.3, de x = +4 a x = −4; el carro del
Master arranca en ella hacia el oeste).

  1. Spawnea AutoModel_Obstacle1 (modelo AutoNOMOS_mini_obstacle) en el
     carril derecho, adelante del carro, mirando al oeste.
  2. Lo maneja en lazo cerrado a 10 Hz con /get_entity_state:
       • dirección: lo mantiene en el centro de su carril (y = lane_y)
       • velocidad: la ajusta para ir a speed_mps (m/s reales)
       • se frena si el carro del Master queda justo enfrente de él
       • al llegar a end_x (antes de la curva) se detiene
  3. Cuando el carro del Master ya no está en esa recta, lo regresa al
     inicio (respawn), para que en la siguiente vuelta lo vuelva a encontrar.

Parámetros (ros2 run ... --ros-args -p speed_mps:=0.10):
  speed_mps   0.12   velocidad del obstáculo (más de ~0.15 m/s ya cuesta rebasarlo)
  start_x     0.5    dónde aparece (el carro arranca en x = 2.5)
  end_x      -3.6    dónde se detiene (la curva empieza en x ≈ −4.0)
  lane_y      3.28   centro del carril derecho de la recta de arriba
  respawn     true   regresarlo al inicio en cada vuelta

Requiere el plugin libgazebo_ros_state (ya está en TMR2021.world).
"""
import math
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, GetEntityState, SetEntityState
from std_msgs.msg import Int16
from ament_index_python.packages import get_package_share_directory
import os

OBST = 'AutoModel_Obstacle1'
CAR = 'AutoModelMini'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class MovingObstacle(Node):
    def __init__(self):
        super().__init__('spawn_moving_obstacle')
        self.declare_parameter('speed_mps', 0.12)
        self.declare_parameter('start_x', 0.5)
        self.declare_parameter('end_x', -3.6)
        self.declare_parameter('lane_y', 3.28)
        self.declare_parameter('respawn', True)
        g = lambda n: self.get_parameter(n).value
        self.v_ref = float(g('speed_mps'))
        self.x0 = float(g('start_x'))
        self.x_end = float(g('end_x'))
        self.y_lane = float(g('lane_y'))
        self.respawn = bool(g('respawn'))

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.pub_speed = self.create_publisher(Int16, f'/{OBST}/manual_control/speed', 1)
        self.pub_steer = self.create_publisher(Int16, f'/{OBST}/manual_control/steering', 1)
        self.cmd = -max(60.0, self.v_ref / 0.00122)      # arranque; luego se corrige con la velocidad medida

    # ── Servicios ─────────────────────────────────────────────────────────
    def call(self, cli, req, timeout=3.0):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def state(self, name):
        req = GetEntityState.Request(); req.name = name; req.reference_frame = 'world'
        r = self.call(self.get_cli, req, 1.0)
        if r is None or not r.success:
            return None
        p = r.state.pose
        t = r.header.stamp.sec + r.header.stamp.nanosec * 1e-9     # tiempo de SIMULACIÓN
        return p.position.x, p.position.y, yaw_of(p.orientation), t

    def spawn(self):
        models = os.path.join(get_package_share_directory('autonomos_gazebo_simulation'), 'models')
        req = SpawnEntity.Request()
        req.name = OBST
        with open(os.path.join(models, 'AutoNOMOS_mini_obstacle', 'model.sdf')) as f:
            req.xml = f.read()
        req.initial_pose.position.x = self.x0
        req.initial_pose.position.y = self.y_lane
        req.initial_pose.position.z = 0.17
        req.initial_pose.orientation.z = 1.0      # yaw = 180° (hacia el oeste)
        req.initial_pose.orientation.w = 0.0
        r = self.call(self.spawn_cli, req, 20.0)
        ok = r is not None and r.success
        self.get_logger().info(f'Spawneado: {OBST}' if ok else f'No se pudo spawnear {OBST}: {r}')
        return ok

    def teleport_to_start(self):
        req = SetEntityState.Request()
        req.state.name = OBST
        req.state.reference_frame = 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = self.x0, self.y_lane, 0.17
        req.state.pose.orientation.z, req.state.pose.orientation.w = 1.0, 0.0
        self.call(self.set_cli, req, 2.0)
        self.get_logger().info('Obstaculo de regreso al inicio de la recta')

    def send(self, speed, steer):
        self.pub_speed.publish(Int16(data=int(speed)))
        self.pub_steer.publish(Int16(data=int(steer)))

    # ── Lazo ──────────────────────────────────────────────────────────────
    def run(self):
        for cli, n in ((self.spawn_cli, '/spawn_entity'), (self.get_cli, '/get_entity_state'),
                       (self.set_cli, '/set_entity_state')):
            self.get_logger().info(f'Esperando {n}...')
            cli.wait_for_service()
        if not self.spawn():
            return
        self.get_logger().info(
            f'Obstaculo movil: {self.v_ref:.2f} m/s, de x={self.x0:.1f} a x={self.x_end:.1f} (y={self.y_lane:.2f})')

        prev = None
        v_f = 0.0
        stopped_at_end = False
        while rclpy.ok():
            t0 = time.monotonic()
            o = self.state(OBST)
            c = self.state(CAR)
            if o is None:
                time.sleep(0.1)
                continue
            x, y, yaw, now = o
            if prev is not None and now - prev[2] > 0.05:
                v_meas = math.hypot(x - prev[0], y - prev[1]) / (now - prev[2])
                v_f = 0.7 * v_f + 0.3 * v_meas
            prev = (x, y, now)

            # ¿El carro del Master está justo enfrente del obstáculo (mismo carril)?
            blocked = False
            car_on_straight = False
            if c is not None:
                cx, cy, _, _ = c
                car_on_straight = abs(cy - self.y_lane) < 0.6 and -4.2 < cx < 4.2
                ahead = -(cx - x)                     # el obstáculo va hacia −x
                if 0.0 < ahead < 0.75 and abs(cy - y) < 0.25:
                    blocked = True

            if x <= self.x_end:
                stopped_at_end = True

            # Dirección: centro del carril y rumbo hacia el oeste (yaw = 180°)
            # (>90 = izquierda; para quien va al oeste, la izquierda es −y)
            e_y = y - self.y_lane                    # + = se fue a su derecha (norte)
            e_h = math.degrees(wrap(yaw - math.pi))  # + = apunta a su izquierda
            steer = max(60, min(120, round(90 + 150.0 * e_y - 1.2 * e_h)))

            if stopped_at_end or blocked:
                speed = 0
            else:
                # velocidad: corrige el comando con lo medido (Gazebo no es lineal a baja velocidad)
                # (integral: ~300 unidades de comando por segundo por cada m/s de error)
                self.cmd -= 30.0 * (self.v_ref - v_f)
                self.cmd = max(-250.0, min(-50.0, self.cmd))
                speed = round(self.cmd)
            self.send(speed, steer)

            # Regresar al inicio cuando el carro ya no está en la recta
            if self.respawn and stopped_at_end and not car_on_straight:
                self.send(0, 90)
                self.teleport_to_start()
                stopped_at_end = False
                prev = None
                v_f = 0.0

            time.sleep(max(0.0, 0.1 - (time.monotonic() - t0)))


def main():
    rclpy.init()
    node = MovingObstacle()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(0, 90)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
