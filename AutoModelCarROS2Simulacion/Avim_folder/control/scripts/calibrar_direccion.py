"""
calibrar_direccion.py
════════════════════════════════════════════════════════════════
Mide en Gazebo CÓMO gira la dirección durante la maniobra de
estacionamiento, para que Master_parking_v2 (sin IMU) estime bien el
rumbo.

En Gazebo la dirección la mueve un PID lento (autonomos_plugin): con el
carro detenido casi no gira, y rodando tarda en llegar al tope. Por eso
un arco corto gira mucho menos de lo que da el radio de giro "estable"
que mide calibrar_giro.py.

Hace exactamente lo que hace el Master: dirección al tope con el carro
detenido (steer_settle_s) y luego reversa a 150 por varios segundos.
Con /get_entity_state mide el ángulo REAL de las ruedas en el tiempo.

Uso:
  1) Simulación SIN Master:
       ros2 launch bring_up sim_parking_v2.launch.py run_master:=false
  2) Otra terminal (source ~/avim_ws/install/setup.bash):
       python3 calibrar_direccion.py
  3) Copia los valores que imprime en PARK_PARAMS de sim_parking_v2.launch.py.
════════════════════════════════════════════════════════════════
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from gazebo_msgs.srv import GetEntityState, SetEntityState

ENTITY = 'AutoModelMini'
WHEELBASE = 0.25
START = (0.0, -6.0, 0.0)      # punto despejado, lejos de la pista
SETTLE = 0.6                  # = steer_settle_s del Master
SPEED = 150                   # = maneuver_cmd del Master (reversa)
RUN = 5.0                     # segundos rodando
SPEED_MPS_PER_CMD = 0.00122   # el que usas (para comparar)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calib(Node):
    def __init__(self):
        super().__init__('calibrar_direccion')
        self.steer_pub = self.create_publisher(Int16, '/AutoModelMini/manual_control/steering', 10)
        self.speed_pub = self.create_publisher(Int16, '/AutoModelMini/manual_control/speed', 10)
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        for cli, name in ((self.get_cli, '/get_entity_state'), (self.set_cli, '/set_entity_state')):
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'No existe {name}: ¿está corriendo Gazebo?')

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
        return t, st.pose.position.x, st.pose.position.y, yaw_of(st.pose.orientation)

    def hold(self, steer, speed, secs, log=None):
        """Manda el comando durante 'secs' de tiempo de simulación (y guarda poses si log)."""
        t0 = self.pose()[0]
        while True:
            self.cmd(steer, speed)
            rclpy.spin_once(self, timeout_sec=0.02)
            p = self.pose()
            if log is not None:
                log.append(p)
            if p[0] - t0 >= secs:
                return

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
        # dirección al centro: rodando un poco para que de verdad se enderece
        self.hold(90, -150, 2.0)
        self.hold(90, 0, 1.5)
        fut = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        self.hold(90, 0, 1.0)

    def maneuver(self, steer):
        self.reset()
        self.hold(steer, 0, SETTLE)                 # como la pausa del Master
        log = []
        self.hold(steer, SPEED, RUN, log)
        self.cmd(90, 0)
        return log


def analyze(log):
    """Devuelve dict con delta(t), velocidad, etc."""
    t = [p[0] for p in log]; x = [p[1] for p in log]; y = [p[2] for p in log]; th = [p[3] for p in log]
    # rumbo continuo
    for i in range(1, len(th)):
        while th[i] - th[i - 1] > math.pi: th[i] -= 2 * math.pi
        while th[i] - th[i - 1] < -math.pi: th[i] += 2 * math.pi
    t0 = t[0]
    s = [0.0]
    for i in range(1, len(t)):
        s.append(s[-1] + math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]))
    pts = []      # (tiempo desde el comando, v, delta)
    W = 3
    for i in range(W, len(t) - W):
        dt = t[i + W] - t[i - W]
        if dt <= 0: continue
        v = (s[i + W] - s[i - W]) / dt
        w = (th[i + W] - th[i - W]) / dt
        d = math.atan(WHEELBASE * abs(w) / v) if v > 0.03 else float('nan')
        pts.append((t[i] - t0, v, d))
    tail = [p for p in pts if p[0] > RUN - 1.5 and not math.isnan(p[2])]
    if not tail:
        return None
    v_ss = sum(p[1] for p in tail) / len(tail)
    d_ss = sorted(p[2] for p in tail)[len(tail) // 2]
    # arranque: 63% de la velocidad
    t_v = next((p[0] for p in pts if p[1] >= 0.63 * v_ss), 0.15)
    moving = [p for p in pts if not math.isnan(p[2]) and p[1] > 0.5 * v_ss]
    t_m = moving[0][0] if moving else 0.0
    d0 = moving[0][2] if moving else 0.0
    # ajuste delta(t) = d_ss - (d_ss - d0) exp(-(t - t_m)/tau)
    best = (1e9, 0.15)
    for k in range(1, 121):
        tau = 0.025 * k
        err = 0.0
        for (tt, _, d) in moving:
            if tt - t_m > 3.0: break
            e = d - (d_ss - (d_ss - d0) * math.exp(-(tt - t_m) / tau))
            err += e * e
        best = min(best, (err, tau))
    tau = best[1]
    # ángulo con el que ARRANCÓ a rodar (extrapolado al inicio del movimiento ~ t_v)
    d_start = d_ss - (d_ss - d0) * math.exp(-max(0.0, t_v - t_m) / tau) if t_v > t_m else d0
    # cuánto giró en los primeros 0.35 m (como el primer arco)
    i35 = next((i for i in range(len(s)) if s[i] >= 0.35), None)
    turn35 = abs(th[i35] - th[0]) if i35 else float('nan')
    return dict(v_ss=v_ss, d_ss=d_ss, R_ss=WHEELBASE / math.tan(d_ss), tau=tau,
                frac=max(0.0, min(1.0, d_start / d_ss)), t_v=t_v, turn35=turn35)


def main():
    rclpy.init()
    node = Calib()
    res = []
    for steer, name in ((0, 'derecha'), (180, 'izquierda')):
        print(f'\n── Dirección toda a la {name}, reversa a {SPEED} ──')
        r = analyze(node.maneuver(steer))
        if r is None:
            print('  no se movió'); continue
        res.append(r)
        print(f"  velocidad estable  {r['v_ss']:.3f} m/s  (con speed_mps_per_cmd {SPEED_MPS_PER_CMD} serían {SPEED * SPEED_MPS_PER_CMD:.3f})")
        print(f"  ángulo de rueda    {math.degrees(r['d_ss']):.1f}°  -> radio estable {r['R_ss']:.3f} m")
        print(f"  la dirección tarda {r['tau']:.2f} s (constante de tiempo) en llegar al tope rodando")
        print(f"  al arrancar ya tenía el {100 * r['frac']:.0f}% del ángulo (lo que giró detenido)")
        print(f"  en los primeros 0.35 m giró {math.degrees(r['turn35']):.1f}°  (radio efectivo {0.35 / r['turn35']:.2f} m)")
        print(f"  la velocidad tarda ~{r['t_v']:.2f} s en llegar al 63%")
    node.cmd(90, 0)
    node.destroy_node()
    rclpy.shutdown()
    if not res:
        print('\nNo hubo mediciones.'); return
    n = len(res)
    R = sum(r['R_ss'] for r in res) / n
    tau = sum(r['tau'] for r in res) / n
    frac = sum(r['frac'] for r in res) / n
    vlag = sum(r['t_v'] for r in res) / n
    print('\n════════ Copia esto en PARK_PARAMS (sim_parking_v2.launch.py) ════════')
    print(f"    'turn_radius_m': {R:.3f},")
    print(f"    'steer_lag_s': {tau:.2f},")
    print(f"    'steer_static_frac': {frac:.2f},")
    print(f"    'speed_lag_s': {vlag:.2f},")
    v = sum(r['v_ss'] for r in res) / n
    if abs(v - SPEED * SPEED_MPS_PER_CMD) > 0.1 * v:
        print(f"    # OJO: a {SPEED} va a {v:.3f} m/s; speed_mps_per_cmd debería ser {v / SPEED:.5f} para la maniobra")
    print()


if __name__ == '__main__':
    main()
