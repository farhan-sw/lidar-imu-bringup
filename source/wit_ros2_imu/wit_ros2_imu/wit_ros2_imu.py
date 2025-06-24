import time
import math
import serial
import struct
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile

def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (radians) to quaternion [x, y, z, w].
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]

class IMUDriverNode(Node):
    def __init__(self, port_name):
        super().__init__('imu_driver_node')

        # Frame ID untuk header pesan
        self.frame_id = 'imu_link'

        # Publisher IMU dan Magnetometer
        qos = QoSProfile(depth=10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', qos)

        # Serial port
        self.port_name = port_name
        self.baud_rate = 9600

        # Buffer dan state untuk parsing data serial
        self.buff = {}
        self.key = 0

        # Data sensor
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]

        # Thread untuk membaca data serial
        self.driver_thread = threading.Thread(target=self.driver_loop, daemon=True)
        self.driver_thread.start()

    def handle_serial_data(self, raw_byte):
        """
        Memproses byte data serial satu per satu, mengumpulkan frame data,
        dan mengupdate data sensor jika frame lengkap dan valid.
        """
        # Simpan byte ke buffer
        self.buff[self.key] = raw_byte
        self.key += 1

        # Cek header byte pertama harus 0x55
        if self.buff.get(0, None) != 0x55:
            self.key = 0
            self.buff.clear()
            return False

        # Tunggu sampai buffer cukup panjang (11 bytes)
        if self.key < 11:
            return False

        data_buff = [self.buff[i] for i in range(11)]

        # Cek tipe data berdasarkan byte ke-1
        data_type = data_buff[1]

        # Fungsi untuk parsing data 8 byte mulai dari index 2
        def parse_data():
            return hex_to_short(data_buff[2:10])

        # Cek checksum
        if not check_sum(data_buff[0:10], data_buff[10]):
            self.get_logger().warn(f"Checksum failure for data type 0x{data_type:X}")
            self.key = 0
            self.buff.clear()
            return False

        if data_type == 0x51:  # Acceleration
            raw_acc = parse_data()
            self.acceleration = [x / 32768.0 * 16 * 9.8 for x in raw_acc]

        elif data_type == 0x52:  # Angular velocity
            raw_gyro = parse_data()
            self.angular_velocity = [x / 32768.0 * 2000 * math.pi / 180 for x in raw_gyro]

        elif data_type == 0x53:  # Angle (Euler degree)
            raw_angle = parse_data()
            self.angle_degree = [x / 32768.0 * 180 for x in raw_angle]

        elif data_type == 0x54:  # Magnetometer
            raw_mag = parse_data()
            self.magnetometer = raw_mag

        else:
            # Unknown data type, reset buffer
            self.key = 0
            self.buff.clear()
            return False

        # Reset buffer setelah parsing frame
        self.key = 0
        self.buff.clear()

        # Return True jika data sudut (0x53) sudah diterima, sebagai tanda update lengkap
        return data_type == 0x53

    def driver_loop(self):
        """
        Thread utama untuk membaca data dari serial port dan memprosesnya.
        """
        try:
            ser = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=0.5)
            if ser.isOpen():
                self.get_logger().info(f"Serial port {self.port_name} opened successfully.")
            else:
                ser.open()
                self.get_logger().info(f"Serial port {self.port_name} opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port_name}: {e}")
            return

        while rclpy.ok():
            try:
                buff_count = ser.in_waiting
                if buff_count > 0:
                    buff_data = ser.read(buff_count)
                    for b in buff_data:
                        updated = self.handle_serial_data(b)
                        if updated:
                            self.publish_imu_and_mag()
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

        ser.close()
        self.get_logger().info("Serial port closed.")

    def publish_imu_and_mag(self):
        """
        Membuat dan mempublish pesan IMU dan Magnetometer ke ROS2.
        """
        # Update header timestamp
        now = self.get_clock().now().to_msg()

        # IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id

        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = self.acceleration[0]
        imu_msg.linear_acceleration.y = self.acceleration[1]
        imu_msg.linear_acceleration.z = self.acceleration[2]

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]

        # Orientation quaternion dari Euler angle (radian)
        roll = math.radians(self.angle_degree[0])
        pitch = math.radians(self.angle_degree[1])
        yaw = math.radians(self.angle_degree[2])
        qx, qy, qz, qw = get_quaternion_from_euler(roll, pitch, yaw)

        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        # Publish IMU data
        self.imu_pub.publish(imu_msg)

        # MagneticField message
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = self.frame_id

        # Data magnetometer biasanya dalam unit microtesla (uT)
        # Jika data mentah, bisa dikalibrasi sesuai datasheet sensor
        # Di sini kita asumsikan data sudah dalam uT atau perlu skala sesuai sensor
        mag_msg.magnetic_field.x = float(self.magnetometer[0])
        mag_msg.magnetic_field.y = float(self.magnetometer[1])
        mag_msg.magnetic_field.z = float(self.magnetometer[2])

        # Publish magnetometer data
        self.mag_pub.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    port = '/dev/ttyUSB1'  # Ganti sesuai port Anda
    imu_node = IMUDriverNode(port)
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
