#!/usr/bin/env python3
"""
diagnostico_alto.py — ¿por qué el carro no se detiene en las señales de ALTO?

Revisa, en orden, cada eslabón de la cadena:
  1. ¿Gazebo cargó las señales (stop_sign_1 … stop_sign_8)?
  2. ¿Está corriendo el nodo stop_sign_detector? ¿La cámara publica?
  3. Pone el carro frente a la señal stop_sign_1 (cruce izquierdo, subiendo)
     a 0.8, 1.2 y 1.8 m, toma la imagen de la cámara y:
       • corre la MISMA visión del detector y dice qué vio y por qué
         aceptó/rechazó cada mancha roja
       • cuenta si el nodo stop_sign_detector publicó /stop_sign_position
  4. Guarda imágenes en ~/diag_alto/ (imagen, máscara roja y anotada).

Uso (con la simulación corriendo SIN Master, para que no mueva el carro):
  terminal 1:
    ros2 launch bring_up sim_static_obstacles_v2.launch.py run_master:=false obstacles:=false
  terminal 2:
    source ~/avim_ws/install/setup.bash
    python3 ~/avim_ws/src/AutoModelCarROS2Simulacion/Avim_folder/control/scripts/diagnostico_alto.py
"""
import importlib.util
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int16
from gazebo_msgs.srv import GetEntityState, SetEntityState
from cv_bridge import CvBridge

CAR = 'AutoModelMini'
CAM_TOPIC = '/camera/image_raw'
DEPTH_TOPIC = '/camera/depth/image_raw'
SIGN = ('stop_sign_1', -0.31, -1.67)   # señal del carril que sube por el cruce izquierdo
CAR_X = -0.63                           # centro del carril derecho (subiendo)
OUT = os.path.expanduser('~/diag_alto')


def load_vision():
    """Importa StopSignVision del stop_sign_detector.py INSTALADO."""
    try:
        from ament_index_python.packages import get_package_prefix
        path = os.path.join(get_package_prefix('lane_detection'), 'lib', 'lane_detection', 'stop_sign_detector.py')
    except Exception as e:
        print(f'  [X] No encuentro el paquete lane_detection: {e}')
        return None, None
    path = os.path.realpath(path)
    spec = importlib.util.spec_from_file_location('ssd', path)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as e:
        print(f'  [X] No pude cargar {path}: {e}')
        return None, path
    if not hasattr(mod, 'StopSignVision'):
        print(f'  [X] {path} es el detector VIEJO (no tiene StopSignVision).')
        print('      Copia el stop_sign_detector.py nuevo a Avim_folder/lane_detection/src/ y vuelve a compilar lane_detection.')
        return None, path
    return mod.StopSignVision, path


class Diag(Node):
    def __init__(self):
        super().__init__('diagnostico_alto')
        self.bridge = CvBridge()
        self.img = None
        self.depth = None
        self.n_img = 0
        self.n_pos = 0
        self.last_pos = None
        self.create_subscription(Image, CAM_TOPIC, self.cb_img, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, self.cb_depth, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/stop_sign_position', self.cb_pos, 10)
        self.steer_pub = self.create_publisher(Int16, '/AutoModelMini/manual_control/steering', 10)
        self.speed_pub = self.create_publisher(Int16, '/AutoModelMini/manual_control/speed', 10)
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')

    def cb_img(self, m):
        self.img = self.bridge.imgmsg_to_cv2(m, 'bgr8'); self.n_img += 1

    def cb_depth(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, desired_encoding='passthrough')

    def cb_pos(self, m):
        self.n_pos += 1; self.last_pos = (m.point.x, m.point.y)

    def spin_for(self, secs):
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            a = Int16(); a.data = 90
            s = Int16(); s.data = 0
            self.steer_pub.publish(a); self.speed_pub.publish(s)
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        return fut.result()

    def exists(self, name):
        req = GetEntityState.Request(); req.name = name; req.reference_frame = 'world'
        r = self.call(self.get_cli, req)
        return r is not None and r.success, r

    def place_car(self, x, y, yaw):
        req = SetEntityState.Request()
        req.state.name = CAR
        req.state.reference_frame = 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, 0.17
        req.state.pose.orientation.z = math.sin(yaw / 2)
        req.state.pose.orientation.w = math.cos(yaw / 2)
        r = self.call(self.set_cli, req)
        return r is not None and r.success


def qos_txt(node, topic):
    infos = node.get_publishers_info_by_topic(topic)
    if not infos:
        return 'NADIE publica'
    out = []
    for i in infos:
        rel = str(i.qos_profile.reliability).split('.')[-1]
        out.append(f'{i.node_name} ({rel})')
    return ', '.join(out)


def main():
    rclpy.init()
    d = Diag()
    os.makedirs(OUT, exist_ok=True)
    problems = []

    print('\n══ 1. Señales en Gazebo ══')
    if not d.get_cli.wait_for_service(timeout_sec=10.0):
        print('  [X] No existe /get_entity_state: ¿está corriendo Gazebo con TMR2021.world?')
        return
    found = 0
    for k in range(1, 9):
        ok, _ = d.exists(f'stop_sign_{k}')
        found += ok
    if found == 8:
        print('  [OK] Las 8 señales están cargadas.')
    else:
        print(f'  [X] Solo {found}/8 señales. Gazebo no encontró model://stop_sign_tmr o el mundo es el viejo.')
        print('      • ¿Copiaste TMR2021.world nuevo y la carpeta models/stop_sign_tmr?')
        print('      • Recompila autonomos_gazebo_simulation y haz source install/setup.bash ANTES de lanzar.')
        print('      • Busca en la salida de Gazebo: "Unable to find uri[model://stop_sign_tmr]".')
        problems.append('señales no cargadas')

    print('\n══ 2. Nodos y tópicos ══')
    d.spin_for(1.5)
    nodes = d.get_node_names()
    det_ok = 'stop_sign_detector' in nodes
    print(f'  [{"OK" if det_ok else "X"}] nodo stop_sign_detector {"corriendo" if det_ok else "NO está corriendo (¿lo agregaste al launch?)"}')
    if not det_ok:
        problems.append('detector no corre')
    print(f'  Cámara {CAM_TOPIC}: {qos_txt(d, CAM_TOPIC)}   (imágenes recibidas aquí: {d.n_img})')
    print(f'  Profundidad {DEPTH_TOPIC}: {qos_txt(d, DEPTH_TOPIC)}')
    print(f'  /stop_sign_position la publica: {qos_txt(d, "/stop_sign_position")}')
    subs = d.get_subscriptions_info_by_topic('/stop_sign_position')
    print(f'  /stop_sign_position la escuchan: {", ".join(s.node_name for s in subs if s.node_name != "diagnostico_alto") or "nadie (¿Master nuevo?)"}')
    n_speed = len(d.get_publishers_info_by_topic('/AutoModelMini/manual_control/speed'))
    if n_speed > 1:
        print(f'  [!] Hay {n_speed} nodos mandando velocidad: apaga el Master (run_master:=false) para esta prueba.')
    if d.n_img == 0:
        print('  [X] No llegan imágenes de la cámara.')
        problems.append('sin cámara')

    print('\n══ 3. Carro frente a stop_sign_1 ══')
    Vision, path = load_vision()
    if path:
        print(f'  detector instalado: {path}')
    vis = None
    if Vision:
        vis = Vision({'camera_hfov_deg': 73.8, 'sign_size_m': 0.139, 'side': 'right',
                      'min_lateral_m': 0.10, 'max_lateral_m': 0.65, 'max_distance_m': 2.5,
                      'min_area_px': 250.0, 'aspect_min': 0.55, 'aspect_max': 1.35,
                      'solidity_min': 0.85, 's_min': 100, 'v_min': 70,
                      'h_red_max': 10, 'h_red_min2': 160})
    name, sx, sy = SIGN
    any_seen = False
    for dist in (0.8, 1.2, 1.8):
        # cámara 3 cm detrás del origen del modelo; carro subiendo (yaw = +90°)
        if not d.place_car(CAR_X, sy - dist + 0.03, math.pi / 2):
            print('  [X] No pude mover el carro (/set_entity_state).'); break
        d.n_pos = 0; d.last_pos = None
        d.spin_for(2.0)
        if d.img is None:
            continue
        img = d.img.copy()
        cv2.imwrite(f'{OUT}/camara_{dist:.1f}m.png', img)
        line = f'  a {dist:.1f} m: '
        if vis:
            mask = vis.red_mask(img)
            cv2.imwrite(f'{OUT}/mascara_roja_{dist:.1f}m.png', mask)
            red_px = int(np.count_nonzero(mask))
            res = []
            for cnt, bbox, why in vis.candidates_color_shape(img):
                x, y, w, h = bbox
                if why is None:
                    z, lat, src = vis.locate(img, d.depth, cnt, bbox)
                    why = vis.accept_position(z, lat)
                    txt = f'z={z:.2f} m lat={lat:+.2f} m ({src})'
                else:
                    txt = f'{w}x{h}px'
                ok = why is None
                any_seen |= ok
                res.append(('ACEPTADA ' if ok else f'rechazada [{why}] ') + txt)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0) if ok else (0, 0, 255), 2)
            cv2.imwrite(f'{OUT}/anotada_{dist:.1f}m.png', img)
            line += f'pixeles rojos={red_px}; ' + ('; '.join(res) if res else 'ninguna mancha roja con forma')
            if red_px == 0:
                hsv = cv2.cvtColor(d.img, cv2.COLOR_BGR2HSV)
                print(f'      (máx. saturación en la imagen {hsv[..., 1].max()}, máx. valor {hsv[..., 2].max()})')
        print(line)
        print(f'      nodo stop_sign_detector → /stop_sign_position: {d.n_pos} mensajes'
              + (f', último x={d.last_pos[0]:.2f} y={d.last_pos[1]:+.2f}' if d.last_pos else ''))

    print('\n══ Resumen ══')
    if vis and not any_seen and 'señales no cargadas' not in problems:
        print('  [X] La visión NO acepta la señal: revisa ~/diag_alto/anotada_*.png y mascara_roja_*.png y mándamelas.')
    elif vis and any_seen and d.n_pos == 0 and det_ok:
        print('  [X] La visión SÍ ve la señal pero el nodo no publica: probablemente el nodo no recibe la cámara')
        print('      (QoS) o es el detector viejo. Mira la salida del nodo stop_sign_detector.')
    elif any_seen and d.n_pos > 0:
        print('  [OK] La señal se detecta y se publica. Si el carro no se detiene, el problema está en el Master:')
        print('       debe aparecer "Recibo /stop_sign_position" en su log (¿compilaste control y usas el Master nuevo?).')
    for p in problems:
        print('  - ' + p)
    print(f'  Imágenes en {OUT}/')
    d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
