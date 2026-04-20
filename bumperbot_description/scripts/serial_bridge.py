#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
import math
import time
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Make sure this matches your Jetson's hardware port
        self.port_name = '/dev/myserial'
        self.baudrate = 115200 
        
        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=0.05)
            self.get_logger().info(f"Successfully connected to MCU on {self.port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise SystemExit

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Read buffer for incoming serial data
        self.buffer = bytearray()
        self.timer = self.create_timer(0.02, self.read_serial_data) # 50Hz read loop

    def cmd_vel_callback(self, msg):
        """ Translates /cmd_vel into serial bytes for the MCU """
        linear_v = float(msg.linear.x)
        angular_w = float(msg.angular.z)
        
        # Pack 3 floats (Vx, Vy, Vz) into bytes. (Vy is 0 for standard differential drive)
        data_bytes = struct.pack('<fff', linear_v, 0.0, angular_w)
        
        length = len(data_bytes) + 2 # +1 for Cmd, +1 for Checksum
        cmd_id = 0x01 # 0x01 is the standard ID for Velocity Commands
        
        frame = bytearray([0xFF, 0xFE, length, cmd_id])
        frame.extend(data_bytes)
        
        # Calculate Checksum
        checksum = sum(frame[2:]) & 0xFF
        frame.append(checksum)
        
        self.serial_port.write(frame)

    def read_serial_data(self):
        """ Reads encoder data from MCU, unpacks it, and publishes Odometry """
        if self.serial_port.in_waiting > 0:
            self.buffer.extend(self.serial_port.read(self.serial_port.in_waiting))
            
            # Look for the Start Headers (0xFF 0xFE)
            while len(self.buffer) >= 4:
                if self.buffer[0] == 0xFF and self.buffer[1] == 0xFE:
                    length = self.buffer[2]
                    
                    # Wait until we have the full frame
                    if len(self.buffer) >= length + 3:
                        frame = self.buffer[:length + 3]
                        self.buffer = self.buffer[length + 3:] 
                        self.parse_frame(frame)
                    else:
                        break # Frame incomplete, wait for next cycle
                else:
                    self.buffer.pop(0)

    def parse_frame(self, frame):
        """ Decodes the frame and extracts velocities """
        cmd_id = frame[3]
        payload = frame[4:-1]
        received_checksum = frame[-1]
        
        calculated_checksum = sum(frame[2:-1]) & 0xFF
        if calculated_checksum != received_checksum:
            self.get_logger().warn("Checksum mismatch, dropping frame")
            return
            
        # 0x02 is the standard ID for Odometry Feedback
        if cmd_id == 0x02 and len(payload) == 12: 
            # Unpack 3 floats (Vx, Vy, Vw) from encoders
            vx, vy, vth = struct.unpack('<fff', payload)
            self.publish_odometry(vx, vth)

    def publish_odometry(self, vx, vth):
        """ Calculates math and publishes standard ROS 2 Odometry messages """
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        
        delta_x = (vx * math.cos(self.th)) * dt
        delta_y = (vx * math.sin(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Direct 2D Quaternion Math (Roll=0, Pitch=0, Yaw=self.th)
        q = [0.0, 0.0, math.sin(self.th / 2.0), math.cos(self.th / 2.0)]

        # 1. Publish TF Transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    bridge = SerialBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.serial_port.close()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()