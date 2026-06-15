#!/usr/bin/env python3
# =============================================================
# laser_triangulation_node
# Detecta la linea laser en cada frame, triangula contra el
# plano del laser y publica una nube de puntos (PointCloud2).
#
# Suscribe:
#   image_raw        sensor_msgs/Image
#   camera_info      sensor_msgs/CameraInfo   (fx, fy, cx, cy)
# Publica:
#   laser_points     sensor_msgs/PointCloud2  (la rebanada de este frame)
#
# Parametros:
#   plane           [a, b, c, d]  ecuacion del plano laser a*x+b*y+c*z+d=0
#                                 (EJEMPLO por ahora; va el real tras calibrar)
#   laser_color     'red' | 'green'
#   threshold       umbral de brillo del canal del laser (0..255)
#   min_intensity   brillo minimo para aceptar un pixel como laser
# =============================================================

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header

from cv_bridge import CvBridge
import cv2


class LaserTriangulation(Node):
    def __init__(self):
        super().__init__('laser_triangulation')

        # --- parametros ---
        self.declare_parameter('plane', [0.0, 0.0, 1.0, -0.30])  # EJEMPLO
        self.declare_parameter('laser_color', 'red')
        self.declare_parameter('threshold', 60)
        self.declare_parameter('min_intensity', 40.0)
        self.declare_parameter('frame_id', 'camera')
        # Si la camara NO esta calibrada, usar intrinsecos estimados
        # (aproximado, solo para ver puntos; calibra para medidas reales).
        self.declare_parameter('assume_uncalibrated', True)

        self.plane = np.array(self.get_parameter('plane').value, dtype=float)
        self.laser_color = self.get_parameter('laser_color').value
        self.threshold = int(self.get_parameter('threshold').value)
        self.min_intensity = float(self.get_parameter('min_intensity').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.assume_uncalibrated = bool(self.get_parameter('assume_uncalibrated').value)
        self._warned_estimate = False

        self.bridge = CvBridge()
        self.K = None   # matriz de camara, llega por camera_info

        self.create_subscription(CameraInfo, 'camera_info', self.info_cb, 10)
        self.create_subscription(Image, 'image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(PointCloud2, 'laser_points', 10)

        self.get_logger().info('Nodo de triangulacion listo. Esperando camera_info...')

    def info_cb(self, msg):
        if self.K is None:
            k = np.array(msg.k, dtype=float).reshape(3, 3)
            # camara calibrada -> fx valido (distinto de 0)
            if k[0, 0] > 1.0:
                self.K = k
                self.get_logger().info(
                    f'Intrinsecos (CALIBRADOS): fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
                    f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}')

    def estimate_K(self, w, h):
        # Estimacion gruesa: fx=fy ~ 0.9*ancho (FOV tipico de webcam ~60 grados)
        fx = fy = 0.9 * w
        cx, cy = w / 2.0, h / 2.0
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)

    # ---------- extraccion de la linea laser ----------
    def extract_laser(self, img):
        """Devuelve, por cada fila, la columna subpixel donde esta el laser."""
        if img.ndim == 3:
            b, g, r = cv2.split(img)
            if self.laser_color == 'green':
                chan = cv2.subtract(g, cv2.addWeighted(r, 0.5, b, 0.5, 0))
            else:  # rojo
                chan = cv2.subtract(r, cv2.addWeighted(g, 0.5, b, 0.5, 0))
        else:
            chan = img  # imagen mono

        chan = cv2.GaussianBlur(chan, (5, 5), 0)
        _, mask = cv2.threshold(chan, self.threshold, 255, cv2.THRESH_BINARY)

        rows, cols = [], []
        h = chan.shape[0]
        for y in range(h):
            row = chan[y].astype(np.float32)
            row[mask[y] == 0] = 0.0
            total = row.sum()
            if total < self.min_intensity:
                continue
            # centroide ponderado por brillo -> columna subpixel
            xs = np.arange(row.shape[0], dtype=np.float32)
            cx = float((xs * row).sum() / total)
            rows.append(y)
            cols.append(cx)
        return np.array(cols), np.array(rows, dtype=float)

    # ---------- triangulacion rayo-plano ----------
    def triangulate(self, u, v):
        """u,v = arrays de coords de pixel del laser -> puntos 3D (N,3)."""
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        # rayo normalizado por pixel (direccion desde la camara)
        x = (u - cx) / fx
        y = (v - cy) / fy
        z = np.ones_like(x)
        dirs = np.stack([x, y, z], axis=1)   # (N,3)

        a, b, c, d = self.plane
        n = np.array([a, b, c])
        # interseccion del rayo (origen 0) con el plano: t = -d / (n . dir)
        denom = dirs @ n
        valid = np.abs(denom) > 1e-6
        t = np.zeros_like(denom)
        t[valid] = -d / denom[valid]
        pts = dirs * t[:, None]
        return pts[valid & (t > 0)]

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Si no llego calibracion valida, estimar a partir del tamano de imagen
        if self.K is None:
            if not self.assume_uncalibrated:
                return
            h, w = img.shape[:2]
            self.K = self.estimate_K(w, h)
            if not self._warned_estimate:
                self.get_logger().warn(
                    f'Camara SIN calibrar: usando intrinsecos ESTIMADOS '
                    f'(fx={self.K[0,0]:.0f}). Los puntos apareceran pero las '
                    f'medidas NO son reales hasta calibrar.')
                self._warned_estimate = True
        u, v = self.extract_laser(img)
        if len(u) == 0:
            return
        pts = self.triangulate(u, v)
        if len(pts) == 0:
            return
        self.publish_cloud(pts, msg.header.stamp)

    # ---------- publicar PointCloud2 ----------
    def publish_cloud(self, pts, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        data = pts.astype(np.float32).tobytes()
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = pts.shape[0]
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * pts.shape[0]
        cloud.is_dense = True
        cloud.data = data
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = LaserTriangulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
