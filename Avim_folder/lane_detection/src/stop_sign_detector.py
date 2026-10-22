#!/usr/bin/env python3
"""
STOP Sign Detector  –  ROS 2 Humble   (v2: señales a un LADO del carril)
═══════════════════════════════════════════════════════════════════════
Antes: buscaba un octágono rojo "de frente" en toda la imagen y publicaba
solo True/False. Con la señal a un lado del carril fallaba porque:
  • vista de lado, el octágono se ve angosto (ancho/alto < 0.7) y la
    prueba de forma lo rechazaba;
  • no sabía a qué DISTANCIA estaba la señal, así que el Master se detenía
    en cuanto la veía (a ~2 m), no en el cruce peatonal;
  • aceptaba cualquier cosa roja de la imagen (también las señales del
    carril contrario).

Ahora:
  1. Forma tolerante a perspectiva (octágono visto de lado): ancho/alto
     0.55–1.35, solidez y "extent" altos, 5–12 vértices.
  2. Distancia hacia adelante (m):
       • con imagen de profundidad (depth_topic): mediana de la profundidad
         dentro del octágono  ← la más precisa (Gazebo / Nuwa-HP60C)
       • sin profundidad: por tamaño aparente  z = f · alto_real / alto_px
         (el ALTO del octágono no cambia aunque se vea de lado).
  3. Posición lateral  x = (u - cx) · z / f  (+ = derecha).
     side = 'right' → solo acepta señales a la DERECHA entre
     min_lateral_m y max_lateral_m (descarta las del carril contrario).
  4. Publica:
       /stop_sign_detected   std_msgs/Bool     (igual que antes, en cada cambio)
       /stop_sign_distance   std_msgs/Float32  distancia hacia adelante desde
                                               la CÁMARA (m), cada cuadro con
                                               señal confirmada
       /stop_sign_position   geometry_msgs/PointStamped
                                               x = adelante, y = izquierda (m)
       /stop_sign_detector/debug_image

El Master_overtake usa /stop_sign_distance para frenar JUSTO antes del cruce
(la señal sale del campo de visión antes de llegar; el Master sigue con
odometría).

Cámara simulada: /camera/image_raw (+ /camera/depth/image_raw).
Carro real (Nuwa-HP60C / Astra): /camera/color/image_raw.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError


# ═════════════════════════════════════════════════════════════════════════════
#  Visión (sin ROS, para poder probarla sola)
# ═════════════════════════════════════════════════════════════════════════════
class StopSignVision:
    """Detecta octágonos rojos y estima distancia/posición lateral."""

    def __init__(self, cfg):
        self.cfg = dict(cfg)

    # ── Utilidades ───────────────────────────────────────────────────────
    def focal_px(self, width):
        hfov = math.radians(self.cfg['camera_hfov_deg'])
        return (width / 2.0) / math.tan(hfov / 2.0)

    def red_mask(self, image):
        c = self.cfg
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lo_s, lo_v = int(c['s_min']), int(c['v_min'])
        m1 = cv2.inRange(hsv, np.array([0, lo_s, lo_v]), np.array([int(c['h_red_max']), 255, 255]))
        m2 = cv2.inRange(hsv, np.array([int(c['h_red_min2']), lo_s, lo_v]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(m1, m2)
        k = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        # cerrar huecos (las letras blancas, reflejos) sin pegar objetos vecinos
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        return mask

    def depth_in_contour(self, depth, cnt, img_shape):
        """Mediana de la profundidad (m) dentro del contorno; None si no hay."""
        if depth is None:
            return None
        h_img, w_img = img_shape[:2]
        h_d, w_d = depth.shape[:2]
        c = cnt.astype(np.float32)
        if (h_d, w_d) != (h_img, w_img):            # otra resolución: escalar
            c[:, 0, 0] *= w_d / float(w_img)
            c[:, 0, 1] *= h_d / float(h_img)
        m = np.zeros((h_d, w_d), np.uint8)
        cv2.drawContours(m, [c.astype(np.int32)], 0, 255, -1)
        m = cv2.erode(m, np.ones((3, 3), np.uint8))  # sin las orillas (mezclan fondo)
        vals = depth[m > 0]
        if vals.size < 10:
            return None
        vals = vals.astype(np.float32)
        if depth.dtype == np.uint16:                  # 16UC1 en mm (carro real)
            vals = vals / 1000.0
        vals = vals[np.isfinite(vals) & (vals > 0.10) & (vals < 6.0)]
        if vals.size < 10:
            return None
        return float(np.median(vals))

    # ── Detección ────────────────────────────────────────────────────────
    def candidates_color_shape(self, image):
        """Lista de (contorno, bbox, motivo_rechazo_o_None)."""
        c = self.cfg
        h_img, w_img = image.shape[:2]
        mask = self.red_mask(image)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < c['min_area_px']:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            why = None
            perim = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.03 * perim, True)
            hull = cv2.convexHull(cnt)
            solidity = area / max(cv2.contourArea(hull), 1e-6)
            extent = area / float(w * h)
            aspect = w / float(h)
            circularity = 4.0 * math.pi * area / (perim * perim)
            margin = 2
            if x <= margin or y <= margin or x + w >= w_img - margin or y + h >= h_img - margin:
                why = 'borde'                      # cortada: la distancia no sirve
            elif not (c['aspect_min'] <= aspect <= c['aspect_max']):
                why = 'ancho/alto %.2f' % aspect
            elif not (5 <= len(approx) <= 12):
                why = 'vertices %d' % len(approx)
            elif solidity < c['solidity_min']:
                why = 'solidez %.2f' % solidity
            elif extent < 0.60:                    # octágono ≈ 0.83, triángulo ≈ 0.5
                why = 'extent %.2f' % extent
            elif circularity < 0.55:
                why = 'circ %.2f' % circularity
            out.append((cnt, (x, y, w, h), why))
        return out

    def locate(self, image, depth, cnt, bbox):
        """(z, lateral_derecha, fuente) de un candidato."""
        c = self.cfg
        h_img, w_img = image.shape[:2]
        f = self.focal_px(w_img)
        x, y, w, h = bbox
        z_size = f * c['sign_size_m'] / max(h, 1)
        z = None
        src = 'tam'
        if cnt is not None:
            z_d = self.depth_in_contour(depth, cnt, image.shape)
            # usar la profundidad solo si es coherente con el tamaño
            if z_d is not None and 0.5 * z_size < z_d < 2.0 * z_size:
                z, src = z_d, 'prof'
        if z is None:
            z = z_size
        u = x + w / 2.0
        lat_right = (u - w_img / 2.0) * z / f
        return z, lat_right, src

    def accept_position(self, z, lat_right):
        """Motivo de rechazo por posición, o None si se acepta."""
        c = self.cfg
        if z > c['max_distance_m']:
            return 'lejos %.1fm' % z
        side = c['side']
        if side == 'right':
            if lat_right < c['min_lateral_m']:
                return 'izq %.2fm' % lat_right
            if lat_right > c['max_lateral_m']:
                return 'muy a la der %.2fm' % lat_right
        elif side == 'left':
            if -lat_right < c['min_lateral_m']:
                return 'der %.2fm' % lat_right
            if -lat_right > c['max_lateral_m']:
                return 'muy a la izq %.2fm' % lat_right
        else:
            if abs(lat_right) > c['max_lateral_m']:
                return 'lateral %.2fm' % lat_right
        return None


# ═════════════════════════════════════════════════════════════════════════════
#  Nodo ROS 2
# ═════════════════════════════════════════════════════════════════════════════
class StopSignDetector(Node):

    def __init__(self):
        super().__init__('stop_sign_detector')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('use_yolo', False)
        self.declare_parameter('detection_threshold', 3)   # cuadros seguidos para confirmar
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        # Profundidad alineada con la imagen a color ('' = no usar)
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('camera_hfov_deg', 73.8)    # Gazebo y Nuwa-HP60C
        # ALTO real del octágono rojo (de lado plano a lado plano), en m.
        # Simulación (stop_sign_tmr): 0.139. En el carro real: MEDIRLO.
        self.declare_parameter('sign_size_m', 0.139)
        # Dónde se busca la señal: 'right' (a la derecha del carril, como en
        # la calle), 'left' o 'any'
        self.declare_parameter('side', 'right')
        # (m) a la derecha del centro de la cámara. Negativo: acepta también un poco
        # a la izquierda (saliendo de una curva a la izquierda la señal queda casi
        # enfrente); las del carril contrario quedan a ~0.7 m a la izquierda
        self.declare_parameter('min_lateral_m', -0.15)
        self.declare_parameter('max_lateral_m', 0.65)
        self.declare_parameter('max_distance_m', 2.5)
        self.declare_parameter('min_area_px', 250.0)
        self.declare_parameter('aspect_min', 0.55)         # ancho/alto (de lado se ve angosto)
        self.declare_parameter('aspect_max', 1.35)
        self.declare_parameter('solidity_min', 0.85)
        self.declare_parameter('s_min', 100)               # HSV
        self.declare_parameter('v_min', 70)
        self.declare_parameter('h_red_max', 10)
        self.declare_parameter('h_red_min2', 160)
        self.declare_parameter('yolo_config', '/home/ubuntu/yolo_stop/yolov5s.cfg')
        self.declare_parameter('yolo_weights', '/home/ubuntu/yolo_stop/yolov5s.weights')
        self.declare_parameter('yolo_classes', '/home/ubuntu/yolo_stop/classes.names')

        gp = lambda n: self.get_parameter(n).value
        self.use_yolo = gp('use_yolo')
        self.detection_threshold = int(gp('detection_threshold'))
        cam_topic = gp('camera_topic')
        depth_topic = gp('depth_topic')
        self.vision = StopSignVision({
            'camera_hfov_deg': float(gp('camera_hfov_deg')),
            'sign_size_m': float(gp('sign_size_m')),
            'side': str(gp('side')).lower(),
            'min_lateral_m': float(gp('min_lateral_m')),
            'max_lateral_m': float(gp('max_lateral_m')),
            'max_distance_m': float(gp('max_distance_m')),
            'min_area_px': float(gp('min_area_px')),
            'aspect_min': float(gp('aspect_min')),
            'aspect_max': float(gp('aspect_max')),
            'solidity_min': float(gp('solidity_min')),
            's_min': int(gp('s_min')), 'v_min': int(gp('v_min')),
            'h_red_max': int(gp('h_red_max')), 'h_red_min2': int(gp('h_red_min2')),
        })

        # ── Estado ────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.stop_detected = False
        self.detection_counter = 0
        self.yolo_net = None
        self.depth = None
        self.depth_t = None
        self.z_filt = None

        if self.use_yolo:
            self._load_yolo_model()

        # ── Publishers ────────────────────────────────────────────────────
        self.detection_pub = self.create_publisher(Bool, '/stop_sign_detected', 1)
        self.distance_pub = self.create_publisher(Float32, '/stop_sign_distance', 5)
        self.position_pub = self.create_publisher(PointStamped, '/stop_sign_position', 5)
        self.debug_image_pub = self.create_publisher(Image, '/stop_sign_detector/debug_image', 1)

        # ── Subscribers ───────────────────────────────────────────────────
        # QoS de sensor (best effort): se conecta tanto si la cámara publica
        # "reliable" como "best effort" (con QoS reliable y una cámara best
        # effort el nodo no recibiría NINGUNA imagen, sin dar error)
        self.create_subscription(Image, cam_topic, self.image_callback, qos_profile_sensor_data)
        if depth_topic:
            self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.n_images = 0
        self.create_timer(5.0, self._check_images)

        method = 'YOLO' if self.use_yolo else 'Color+Forma'
        self.get_logger().info(
            f"STOP Sign Detector v2 | {method} | camara: {cam_topic} | "
            f"profundidad: {depth_topic or '(no, distancia por tamano)'} | "
            f"lado: {self.vision.cfg['side']} "
            f"[{self.vision.cfg['min_lateral_m']:.2f}, {self.vision.cfg['max_lateral_m']:.2f}] m | "
            f"senal {self.vision.cfg['sign_size_m'] * 100:.1f} cm")

    # ── YOLO ──────────────────────────────────────────────────────────────
    def _load_yolo_model(self):
        try:
            cfg = self.get_parameter('yolo_config').value
            weights = self.get_parameter('yolo_weights').value
            classes = self.get_parameter('yolo_classes').value
            self.yolo_net = cv2.dnn.readNetFromDarknet(cfg, weights)
            self.yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            with open(classes, 'r') as f:
                self.yolo_classes = [l.strip() for l in f.readlines()]
            self.get_logger().info("Modelo YOLO cargado correctamente")
        except Exception as e:
            self.get_logger().error(f"Error cargando YOLO: {e}")
            self.get_logger().warn("Fallback a deteccion por Color+Forma")
            self.use_yolo = False

    def _yolo_boxes(self, image):
        if self.yolo_net is None:
            return []
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.yolo_net.setInput(blob)
        layer_names = self.yolo_net.getLayerNames()
        output_layers = [layer_names[int(i) - 1]
                         for i in np.array(self.yolo_net.getUnconnectedOutLayers()).flatten()]
        outputs = self.yolo_net.forward(output_layers)
        h, w = image.shape[:2]
        boxes, confs = [], []
        for output in outputs:
            for det in output:
                scores = det[5:]
                class_id = int(np.argmax(scores))
                conf = float(scores[class_id])
                if conf > 0.5 and class_id == 0:
                    bw, bh = int(det[2] * w), int(det[3] * h)
                    boxes.append([int(det[0] * w - bw / 2), int(det[1] * h - bh / 2), bw, bh])
                    confs.append(conf)
        if not boxes:
            return []
        keep = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
        return [tuple(boxes[int(i)]) for i in np.array(keep).flatten()]

    def _check_images(self):
        if self.n_images == 0:
            self.get_logger().warn(f"No llegan imagenes de la camara: revisa camera_topic "
                                   f"({self.get_parameter('camera_topic').value}) con 'ros2 topic list'")

    # ── Profundidad ───────────────────────────────────────────────────────
    def depth_callback(self, msg: Image):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_t = self.get_clock().now()
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge (profundidad): {e}")

    def _fresh_depth(self):
        if self.depth is None or self.depth_t is None:
            return None
        age = (self.get_clock().now() - self.depth_t).nanoseconds * 1e-9
        return self.depth if age < 0.5 else None

    # ── Callback principal ────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        self.n_images += 1
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        V = self.vision
        depth = self._fresh_depth()
        debug = image.copy()

        if self.use_yolo:
            cands = [(None, b, None) for b in self._yolo_boxes(image)]
        else:
            cands = V.candidates_color_shape(image)

        best = None    # (z, lat, src, bbox)
        for cnt, bbox, why in cands:
            x, y, w, h = bbox
            if why is None:
                z, lat, src = V.locate(image, depth, cnt, bbox)
                why = V.accept_position(z, lat)
                label = f'{z:.2f}m {lat:+.2f}m {src}'
            else:
                label = ''
            if why is None:
                if best is None or z < best[0]:
                    best = (z, lat, src, bbox)
            else:
                cv2.rectangle(debug, (x, y), (x + w, y + h), (160, 160, 160), 1)
                cv2.putText(debug, f'{why} {label}', (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        detected = best is not None
        newly_confirmed = False

        # ── Histéresis (igual que antes) ──────────────────────────────────
        if detected:
            self.detection_counter = min(self.detection_counter + 1, self.detection_threshold + 2)
            if self.detection_counter >= self.detection_threshold and not self.stop_detected:
                self.stop_detected = True
                newly_confirmed = True
                self.get_logger().info(f"ALTO confirmado a {best[0]:.2f} m "
                                       f"({best[1]:+.2f} m a la derecha, {best[2]})")
        else:
            if self.detection_counter > 0:
                self.detection_counter -= 1
            if self.stop_detected and self.detection_counter == 0:
                self.stop_detected = False
                self.z_filt = None
                self._publish_detection(False)
                self.get_logger().info("ALTO ya no visible")

        # ── Distancia (solo con señal confirmada y vista en este cuadro) ──
        if detected and self.stop_detected:
            z, lat, src, bbox = best
            # sin filtrar aquí (un filtro retrasa la distancia mientras el carro
            # avanza); el Master promedia las mediciones en su odometría
            self.z_filt = z
            # primero la posición (trae el lateral), luego la distancia
            p = PointStamped()
            p.header = msg.header
            p.point.x = float(self.z_filt)
            p.point.y = float(-lat)           # + = izquierda (convención ROS)
            p.point.z = 0.0
            self.position_pub.publish(p)
            d = Float32()
            d.data = float(self.z_filt)
            self.distance_pub.publish(d)
        # el Bool va DESPUÉS de la distancia: así el Master sabe desde el primer
        # aviso que hay distancia y no hace el alto "inmediato" del modo viejo
        if newly_confirmed:
            self._publish_detection(True)

        # ── Imagen de debug ───────────────────────────────────────────────
        if best is not None:
            z, lat, src, (x, y, w, h) = best
            col = (0, 255, 0) if self.stop_detected else (0, 200, 255)
            cv2.rectangle(debug, (x, y), (x + w, y + h), col, 2)
            cv2.putText(debug, f'ALTO {z:.2f}m  {lat:+.2f}m ({src})', (x - 40, max(15, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
        # zona válida a la distancia de la señal (o a 1 m)
        hh, ww = image.shape[:2]
        f = V.focal_px(ww)
        zr = best[0] if best is not None else 1.0
        if V.cfg['side'] == 'right':
            for lat_lim in (V.cfg['min_lateral_m'], V.cfg['max_lateral_m']):
                u = int(ww / 2 + lat_lim * f / zr)
                if 0 <= u < ww:
                    cv2.line(debug, (u, 0), (u, hh - 1), (255, 128, 0), 1)
        status = 'ALTO!' if self.stop_detected else 'Buscando...'
        color = (0, 255, 0) if self.stop_detected else (0, 0, 255)
        cv2.putText(debug, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.putText(debug, f'Contador: {self.detection_counter}/{self.detection_threshold}'
                    + (f'   z={self.z_filt:.2f} m' if self.z_filt is not None and self.stop_detected else ''),
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        try:
            dm = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            dm.header = msg.header
            self.debug_image_pub.publish(dm)
        except CvBridgeError as e:
            self.get_logger().error(f"Debug image error: {e}")

    def _publish_detection(self, state: bool):
        msg = Bool()
        msg.data = state
        self.detection_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando stop_sign_detector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
