#!/usr/bin/env python3
"""
STOP Sign Detector  –  ROS 2 (Humble / Iron)
Cámara: Orbbec Astra Pro  →  tópico /camera/color/image_raw

Migración ROS 1 → ROS 2:
  • rospy                   → rclpy
  • rospy.init_node(...)    → Node.__init__
  • rospy.get_param('~p')   → self.declare_parameter / self.get_parameter
  • rospy.Subscriber(...)   → self.create_subscription(...)
  • rospy.Publisher(...)    → self.create_publisher(...)
  • rospy.loginfo/logerr    → self.get_logger().info/error
  • rospy.ROSInterruptException → KeyboardInterrupt / rclpy.shutdown()
  • CvBridge sin cambios en API, sí cambia el import del paquete cv_bridge
"""

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError


class StopSignDetector(Node):

    def __init__(self):
        # ROS 1: rospy.init_node('stop_sign_detector', anonymous=False)
        # ROS 2: super().__init__('stop_sign_detector')
        super().__init__('stop_sign_detector')

        # ── Parámetros ────────────────────────────────────────────────────────
        # ROS 1: rospy.get_param('~use_yolo', False)
        # ROS 2: declare_parameter + get_parameter
        self.declare_parameter('use_yolo',      False)
        self.declare_parameter('detection_threshold', 3)
        # Tópico por defecto: color de la Orbbec Astra Pro
        self.declare_parameter('camera_topic',  '/camera/color/image_raw')
        self.declare_parameter('yolo_config',   '/home/ubuntu/yolo_stop/yolov5s.cfg')
        self.declare_parameter('yolo_weights',  '/home/ubuntu/yolo_stop/yolov5s.weights')
        self.declare_parameter('yolo_classes',  '/home/ubuntu/yolo_stop/classes.names')

        self.use_yolo            = self.get_parameter('use_yolo').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        cam_topic                = self.get_parameter('camera_topic').value

        # ── Estado ────────────────────────────────────────────────────────────
        self.bridge             = CvBridge()
        self.stop_detected      = False
        self.detection_counter  = 0
        self.yolo_net           = None

        if self.use_yolo:
            self._load_yolo_model()

        # ── Publishers ────────────────────────────────────────────────────────
        # ROS 1: rospy.Publisher(topic, Type, queue_size=N)
        # ROS 2: self.create_publisher(Type, topic, N)
        self.detection_pub   = self.create_publisher(Bool,  '/stop_sign_detected',              1)
        self.debug_image_pub = self.create_publisher(Image, '/stop_sign_detector/debug_image',  1)

        # ── Subscriber ────────────────────────────────────────────────────────
        # ROS 1: rospy.Subscriber(topic, Type, cb, queue_size=N)
        # ROS 2: self.create_subscription(Type, topic, cb, N)
        self.create_subscription(Image, cam_topic, self.image_callback, 1)

        method = 'YOLO' if self.use_yolo else 'Color+Shape'
        self.get_logger().info(
            f"STOP Sign Detector (ROS 2) | Método: {method} | "
            f"Cámara: {cam_topic}  [Orbbec Astra Pro]")

    # ── YOLO ──────────────────────────────────────────────────────────────────
    def _load_yolo_model(self):
        try:
            cfg     = self.get_parameter('yolo_config').value
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
            self.get_logger().warn("Fallback a detección por Color+Shape")
            self.use_yolo = False

    # ── Callback principal ─────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        if self.use_yolo:
            detected, debug_image = self._detect_yolo(cv_image)
        else:
            detected, debug_image = self._detect_color_shape(cv_image)

        # ── Lógica de umbral con histéresis ───────────────────────────────
        if detected:
            self.detection_counter = min(self.detection_counter + 1,
                                         self.detection_threshold + 2)
            if (self.detection_counter >= self.detection_threshold
                    and not self.stop_detected):
                self.stop_detected = True
                self._publish_detection(True)
                self.get_logger().info("¡STOP SIGN CONFIRMADO!")
        else:
            if self.detection_counter > 0:
                self.detection_counter -= 1
            if self.stop_detected and self.detection_counter == 0:
                self.stop_detected = False
                self._publish_detection(False)
                self.get_logger().info("STOP sign ya no visible")

        # ── Imagen de debug ───────────────────────────────────────────────
        if debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                # Preservar el timestamp original de la cámara
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Debug image error: {e}")

    # ── Detección por Color + Shape ────────────────────────────────────────
    def _detect_color_shape(self, image):
        debug   = image.copy()
        detected = False

        hsv    = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask1  = cv2.inRange(hsv, np.array([0,   100, 100]),
                                  np.array([10,  255, 255]))
        mask2  = cv2.inRange(hsv, np.array([160, 100, 100]),
                                  np.array([180, 255, 255]))
        mask   = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue

            epsilon = 0.04 * cv2.arcLength(cnt, True)
            approx  = cv2.approxPolyDP(cnt, epsilon, True)

            if 6 <= len(approx) <= 10:
                perim       = cv2.arcLength(cnt, True)
                circularity = 4 * np.pi * area / (perim * perim)
                if circularity > 0.5:
                    x, y, w, h = cv2.boundingRect(cnt)
                    if 0.7 < float(w) / h < 1.3:
                        detected = True
                        cv2.drawContours(debug, [approx], 0, (0, 255, 0), 3)
                        cv2.rectangle(debug, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.putText(debug, 'STOP', (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        status = 'STOP DETECTED!' if detected else 'Scanning...'
        color  = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(debug, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.putText(debug,
                    f'Counter: {self.detection_counter}/{self.detection_threshold}',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return detected, debug

    # ── Detección YOLO ────────────────────────────────────────────────────
    def _detect_yolo(self, image):
        debug    = image.copy()
        detected = False
        if self.yolo_net is None:
            return False, debug

        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416),
                                     swapRB=True, crop=False)
        self.yolo_net.setInput(blob)
        layer_names   = self.yolo_net.getLayerNames()
        output_layers = [layer_names[i - 1]
                         for i in self.yolo_net.getUnconnectedOutLayers()]
        outputs = self.yolo_net.forward(output_layers)

        h, w = image.shape[:2]
        for output in outputs:
            for det in output:
                scores     = det[5:]
                class_id   = int(np.argmax(scores))
                confidence = float(scores[class_id])
                if confidence > 0.5 and class_id == 0:
                    detected = True
                    cx = int(det[0] * w); cy = int(det[1] * h)
                    bw = int(det[2] * w); bh = int(det[3] * h)
                    x  = int(cx - bw / 2); y  = int(cy - bh / 2)
                    cv2.rectangle(debug, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
                    cv2.putText(debug, f'STOP {confidence:.2f}', (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return detected, debug

    def _publish_detection(self, state: bool):
        msg = Bool()
        msg.data = state
        self.detection_pub.publish(msg)


def main(args=None):
    # ROS 1: detector = StopSignDetector(); detector.run() [rospy.spin()]
    # ROS 2: rclpy.init() → spin(node) → shutdown()
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
