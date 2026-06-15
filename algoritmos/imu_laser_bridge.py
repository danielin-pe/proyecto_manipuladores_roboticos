#!/usr/bin/env python3
# =============================================================
# Puente serial ESP32-C3 <-> ROS 2 Jazzy
#
# Publica:
#   imu/data_raw   sensor_msgs/Imu           (acel + giro, sin orientacion)
#   imu/mag        sensor_msgs/MagneticField (en Tesla)
#
# Se suscribe a:
#   laser1         std_msgs/UInt8   duty 0..255  (0 = apagado)
#   laser2         std_msgs/UInt8   duty 0..255
#
# Parametros:
#   port      (str)  puerto serial        [/dev/ttyACM0]
#   baud      (int)  baudios              [115200]
#   frame_id  (str)  frame del IMU        [imu_link]
#   rate_hz   (int)  tasa del stream IMU  [100]
#
# Uso:
#   ros2 run scanner_bridge imu_laser_bridge --ros-args -p port:=/dev/ttyACM0
#
# Requisitos:  pip install pyserial  (o sudo apt install python3-serial)
#              y el usuario en el grupo 'dialout'
# =============================================================

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import UInt8

import serial


class ImuLaserBridge(Node):
    def __init__(self):
        super().__init__('imu_laser_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 100)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = int(self.get_parameter('rate_hz').value)

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 50)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 50)

        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.get_logger().info(f'Conectado a {port} @ {baud}')

        self._lock = threading.Lock()
        # Fijar la tasa del firmware al arrancar
        self._send(f'RATE,{rate_hz}')
        self._send('STREAM,1')

        self.create_subscription(UInt8, 'laser1', self.laser1_cb, 10)
        self.create_subscription(UInt8, 'laser2', self.laser2_cb, 10)

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    # ---- envio de comandos ----
    def _send(self, line: str):
        with self._lock:
            self.ser.write((line + '\n').encode('ascii'))

    def laser1_cb(self, msg):
        self._send(f'L,1,{int(msg.data)}')

    def laser2_cb(self, msg):
        self._send(f'L,2,{int(msg.data)}')

    # ---- lectura del stream ----
    def _read_loop(self):
        while self._running and rclpy.ok():
            try:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception as e:
                self.get_logger().warn(f'serial read error: {e}')
                continue

            if not raw or not raw.startswith('I,'):
                continue

            parts = raw.split(',')
            if len(parts) != 12:
                continue
            try:
                v = [float(x) for x in parts[1:]]   # [t_us, ax..az, gx..gz, mx..mz, temp]
            except ValueError:
                continue

            self._publish(v)

    def _publish(self, v):
        # Sello de tiempo de recepcion (tiempo de la Pi).
        # Para fusion mas fina se podria estimar el offset con v[0] (micros del ESP32).
        now = self.get_clock().now().to_msg()

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.frame_id
        imu.linear_acceleration.x = v[1]
        imu.linear_acceleration.y = v[2]
        imu.linear_acceleration.z = v[3]
        imu.angular_velocity.x = v[4]
        imu.angular_velocity.y = v[5]
        imu.angular_velocity.z = v[6]
        # Sin orientacion -> convencion REP 145
        imu.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = self.frame_id
        mag.magnetic_field.x = v[7] * 1e-6   # uT -> T
        mag.magnetic_field.y = v[8] * 1e-6
        mag.magnetic_field.z = v[9] * 1e-6
        self.mag_pub.publish(mag)

    def destroy_node(self):
        self._running = False
        try:
            # Apagar lasers al cerrar (seguridad)
            self._send('L,1,0')
            self._send('L,2,0')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ImuLaserBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
