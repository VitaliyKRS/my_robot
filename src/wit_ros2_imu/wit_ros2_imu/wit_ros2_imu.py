import math
import serial
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from time import sleep

key = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]

def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

def handle_serial_data(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity
    angle_flag = False
    buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return

    if key < 11:
        return
    else:
        data_buff = list(buff.values())
        if buff[1] == 0x51:
            if check_sum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
        elif buff[1] == 0x52:
            if check_sum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
        elif buff[1] == 0x53:
            if check_sum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                angle_flag = True
        elif buff[1] == 0x54:
            if check_sum(data_buff[0:10], data_buff[10]):
                magnetometer = hex_to_short(data_buff[2:10])
        else:
            buff = {}
            key = 0

        buff = {}
        key = 0
        return angle_flag

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]

class IMUDriverNode(Node):
    def __init__(self, port_name):
        super().__init__('imu_driver_node')

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'
        self.mag_msg = MagneticField()
        self.mag_msg.header.frame_id = 'imu_link'

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        try:
            wt_imu = serial.Serial(port="/dev/imu_usb", baudrate=230400, timeout=0.5)
            if wt_imu.isOpen():
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
            else:
                wt_imu.open()
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
        except Exception as e:
            print(e)
            self.get_logger().info("\033[31mSerial port opening failure\033[0m")
            exit(0)

        while True:
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu disconnect")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count)
                    for i in range(0, buff_count):
                        tag = handle_serial_data(buff_data[i])
                        if tag:
                            self.imu_raw_plus_mag()
            sleep(0.01)

    def imu_raw_plus_mag(self):
        accel_x, accel_y, accel_z = acceleration[0], acceleration[1], acceleration[2]
        accel_scale = 16 / 32768.0
        accel_x, accel_y, accel_z = accel_x * accel_scale, accel_y * accel_scale, accel_z * accel_scale

        gyro_x, gyro_y, gyro_z = angularVelocity[0], angularVelocity[1], angularVelocity[2]
        gyro_scale = 2000 / 32768.0
        gyro_x, gyro_y, gyro_z = math.radians(gyro_x * gyro_scale), math.radians(gyro_y * gyro_scale), math.radians(gyro_z * gyro_scale)

        stamp = self.get_clock().now().to_msg()

        self.imu_msg.header.stamp = stamp
        self.imu_msg.header.frame_id = "imu_link"

        self.mag_msg.header.stamp = stamp
        self.mag_msg.header.frame_id = "imu_link"

        angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
        qua = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

        self.imu_msg.orientation.x = qua[0]
        self.imu_msg.orientation.y = qua[1]
        self.imu_msg.orientation.z = qua[2]
        self.imu_msg.orientation.w = qua[3]

        self.imu_msg.angular_velocity.x = float(angularVelocity[0]) #gyro_x
        self.imu_msg.angular_velocity.y = float(angularVelocity[1]) #gyro_y
        self.imu_msg.angular_velocity.z = float(angularVelocity[2]) #gyro_z

        self.imu_msg.linear_acceleration.x = float(acceleration[0]) #accel_x
        self.imu_msg.linear_acceleration.y = float(acceleration[1]) #accel_y
        self.imu_msg.linear_acceleration.z = float(acceleration[2]) #accel_z

        self.mag_msg.magnetic_field.x = float(magnetometer[0]) #magnetometer[0]
        self.mag_msg.magnetic_field.y = float(magnetometer[1])
        self.mag_msg.magnetic_field.z = float(magnetometer[2])

        self.imu_pub.publish(self.imu_msg)
        self.mag_pub.publish(self.mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUDriverNode('/dev/ttyACM0')

    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

