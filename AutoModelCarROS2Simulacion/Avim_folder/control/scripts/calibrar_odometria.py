#!/usr/bin/env python3
"""
calibrar_odometria.py
════════════════════════════════════════════════════════════════
Mide en Gazebo cómo responde DE VERDAD el carro a los comandos:
  • m/s por unidad de comando de velocidad   -> speed_mps_per_cmd
  • grados de rueda por unidad de dirección  -> steer_deg_per_cmd, steer_offset_deg
                                                max_steer_deg

Sin IMU, Master_overtake calcula hacia dónde va el carro solo con los
comandos que manda. Si estos números están mal, durante el rebase (cuando
el detector pierde las líneas) el carro se desvía. Con estos valores
medidos la odometría queda bien.

Usa la posición REAL del carro que da Gazebo (/get_entity_state), así que
solo sirve en simulación. En el carro real: mide el radio de un círculo
completo con la dirección fija (ángulo = atan(0.25 / radio)).

Uso:
  1) Lanza la simulación SIN el Master y SIN obstáculos:
       ros2 launch bring_up sim_static_obstacles_v2.launch.py run_master:=false obstacles:=false
  2) En otra terminal (con source ~/avim_ws/install/setup.bash):
       python3 calibrar_odometria.py
  3) Copia los valores que imprime en MASTER_PARAMS del launch.

El carro hará círculos fuera de la pista (el piso de Gazebo es infinito).
════════════════════════════════════════════════════════════════
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from gazebo_msgs.srv import GetEntityState, SetEntityState

ENTITY = 'AutoModelMini'
WHEELBASE = 0.25
START = (0.0, -6.0, 0.0)          # punto despejado, lejos de la pista


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calib(Node):
    def __init__(self):
        super().__init__('calibrar_odometria')
        self.steer_pub = self.create_publisher(Int16, '/AutoModelMini/manual_control/steering', 10)
        self.speed_pub = self.create_publisher(Int16, '/AutoModelMini/manual_control/speed', 10)
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        for cli, name in ((self.get_cli, '/get_entity_state'), (self.set_cli, '/set_entity_state')):
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'No existe {name}: ¿está corriendo Gazebo con el mundo TMR2021?')

    def cmd(self, steer, speed):
        a = Int16(); a.data = int(steer)
        s = Int16(); s.data = int(speed)
        self.steer_pub.publish(a)
        self.speed_pub.publish(s)

    def pose(self):
        req = GetEntityState.Request(); req.name = ENTITY; req.reference_frame = 'world'
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        st = res.state
        t = res.header.stamp.sec + res.header.stamp.nanosec * 1e-9     # tiempo de SIMULACIÓN
        return st.pose.position.x, st.pose.position.y, yaw_of(st.pose.orientation), t

    def reset(self):
        self.cmd(90, 0)
        req = SetEntityState.Request()
        req.state.name = ENTITY
        req.state.reference_frame = 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = START[0], START[1], 0.17
        req.state.pose.orientation.z = math.sin(START[2] / 2)
        req.state.pose.orientation.w = math.cos(START[2] / 2)
        fut = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        self.hold(90, 0, 1.5)

    def hold(self, steer, speed, secs):
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            self.cmd(steer, speed)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_test(self, steer, speed, settle=2.0, measure=2.5):
        """Devuelve (velocidad m/s, giro rad/s) en régimen permanente."""
        self.reset()
        self.hold(steer, speed, settle)
        x0, y0, th0, t0 = self.pose()
        n = int(measure / 0.25)
        path = 0.0; dth = 0.0; px, py, pth = x0, y0, th0
        for _ in range(n):
            self.hold(steer, speed, 0.25)
            x, y, th, t = self.pose()
            path += math.hypot(x - px, y - py)
            d = th - pth
            while d > math.pi: d -= 2 * math.pi
            while d < -math.pi: d += 2 * math.pi
            dth += d
            px, py, pth = x, y, th
        dt = t - t0
        self.cmd(90, 0)
        return path / dt, dth / dt


def main():
    rclpy.init()
    node = Calib()
    print('\n── 1) Velocidad (dirección al centro) ──')
    ratios = []
    for spd in (-200, -300, -400):
        v, w = node.run_test(90, spd)
        ratios.append(v / abs(spd))
        print(f'  comando {spd:5d} -> {v:.3f} m/s  ({v / abs(spd):.5f} m/s por unidad)')
    k_speed = sum(ratios) / len(ratios)

    print('\n── 2) Dirección (velocidad -250) ──')
    pts = []
    for st in (55, 70, 80, 100, 110, 125):
        v, w = node.run_test(st, -250)
        if v < 0.05:
            print(f'  steer {st}: el carro casi no se movió, se omite'); continue
        delta = math.degrees(math.atan(WHEELBASE * w / v))
        pts.append((st - 90, delta))
        print(f'  steer {st:3d} -> giro {math.degrees(w):6.1f} °/s, v {v:.3f} m/s -> rueda {delta:+6.2f}°')

    node.cmd(90, 0)
    node.destroy_node()
    rclpy.shutdown()

    if len(pts) < 3:
        print('\nNo hubo suficientes mediciones.'); return
    n = len(pts)
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
    sxx = sum(p[0] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    a = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    b = (sy - a * sx) / n
    print('\n════════ Copia esto en MASTER_PARAMS ════════')
    print(f"    'speed_mps_per_cmd': {k_speed:.5f},")
    print(f"    'steer_deg_per_cmd': {a:.4f},")
    print(f"    'steer_offset_deg': {b:.3f},")
    print(f"    'max_steer_deg': {abs(a) * 88.0:.1f},")
    print("    'auto_steer_gain': False,      # ya está calibrada")
    print("    'auto_speed_scale': False,")
    print('(valores que usa el Master por defecto: 0.00121 / 0.2526 / 0.573 / 22.0)\n')


if __name__ == '__main__':
    main()
