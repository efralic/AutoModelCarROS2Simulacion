#!/usr/bin/env python3
"""
calibrar_giro.py
════════════════════════════════════════════════════════════════
Mide en Gazebo el RADIO DE GIRO MÍNIMO (dirección toda a un lado), que es
lo que usa Master_parking_v2 para planear la maniobra de estacionamiento.

calibrar_odometria.py solo midió la dirección hasta ±35 comandos; con la
dirección al tope el mecanismo puede girar menos (o más) de lo que da la
recta extrapolada.

Uso:
  1) Simulación SIN Master:
       ros2 launch bring_up sim_parking_v2.launch.py run_master:=false
  2) Otra terminal (source ~/avim_ws/install/setup.bash):
       python3 calibrar_giro.py
  3) Copia 'turn_radius_m' en PARK_PARAMS de sim_parking_v2.launch.py.

El carro hace círculos fuera de la pista (el piso de Gazebo es infinito).
En el carro real: con la dirección al tope, marca en el piso el círculo
que describe el centro del eje trasero y mide su radio.
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
    print('\n── Radio de giro con la dirección al tope (velocidad 150) ──')
    res = {}
    for steer, name in ((0, 'derecha'), (180, 'izquierda')):
        for spd, sentido in ((150, 'reversa'), (-150, 'adelante')):
            v, w = node.run_test(steer, spd, settle=2.0, measure=2.5)
            if abs(w) < 1e-3:
                print(f'  {name:9s} {sentido:8s}: no giró'); continue
            R = v / abs(w)
            res[(name, sentido)] = R
            print(f'  {name:9s} {sentido:8s}: v={v:.3f} m/s  giro={math.degrees(w):+6.1f} °/s  ->  radio {R:.3f} m'
                  f'  (rueda {math.degrees(math.atan(WHEELBASE / R)):.1f}°)')
    node.cmd(90, 0)
    node.destroy_node()
    rclpy.shutdown()
    rev = [R for (n, s), R in res.items() if s == 'reversa']
    if not rev:
        print('\nNo hubo mediciones.'); return
    R = sum(rev) / len(rev)
    print('\n════════ Copia esto en PARK_PARAMS (sim_parking_v2.launch.py) ════════')
    print(f"    'turn_radius_m': {R:.3f},")
    print("    'steer_right_cmd': 0.0,")
    print("    'steer_left_cmd': 180.0,")
    if len(rev) == 2 and abs(rev[0] - rev[1]) > 0.08 * R:
        print('(OJO: gira distinto a la derecha que a la izquierda; se usa el promedio)')
    print()


if __name__ == '__main__':
    main()
