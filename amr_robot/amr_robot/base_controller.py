#!/usr/bin/env python3
"""
Debug version - Add logging to verify TF timer rate
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import serial
import time
import threading
import math

# ================= ROBOT PARAMETERS =================
WHEEL_DIAMETER = 0.127
WHEEL_BASE = 0.46
LEFT_PPR = 2320
RIGHT_PPR = 2350
MAX_LINEAR = 0.5
MAX_ANGULAR = 2.0
SERIAL_TIMEOUT = 1.0
TF_PUBLISH_RATE = 50.0

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_rate', 10.0)
        self.declare_parameter('tf_rate', 50.0)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.cmd_rate = self.get_parameter('cmd_rate').value
        self.tf_rate = self.get_parameter('tf_rate').value
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_linear = 0.0
        self.v_angular = 0.0
        
        # Encoder counts
        self.left_ticks = 0
        self.right_ticks = 0
        
        # Command velocities
        self.target_v = 0.0
        self.target_w = 0.0
        self.last_cmd_time = time.time()
        
        # Serial connection
        self.serial = None
        self.connected = False
        self.running = False
        self.shutting_down = False
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Track last odom update time
        self.last_odom_update = time.time()
        self.odom_stale_warned = False
        
        # DEBUG: Counter for TF publishes
        self.tf_publish_count = 0
        self.tf_timer_start = time.time()
        
        # ROS2 Publishers - CREATE THESE FIRST!
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ROS2 Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # CRITICAL: Two separate timers
        self.create_timer(1.0 / self.cmd_rate, self.cmd_loop)
        self.create_timer(1.0 / self.tf_rate, self.tf_loop)
        
        # DEBUG: Timer to report TF rate
        self.create_timer(5.0, self.report_tf_rate)
        
        # Connect to Arduino AFTER everything else is initialized
        self.connect_serial(port, baud)
        
        self.get_logger().info("========================================")
        self.get_logger().info(f"Base controller DEBUG VERSION")
        self.get_logger().info(f"Serial: {port} @ {baud}")
        self.get_logger().info(f"Command rate: {self.cmd_rate} Hz")
        self.get_logger().info(f"TF publish rate: {self.tf_rate} Hz")
        self.get_logger().info("========================================")
    
    def report_tf_rate(self):
        """DEBUG: Report actual TF publishing rate"""
        elapsed = time.time() - self.tf_timer_start
        if elapsed > 0:
            actual_rate = self.tf_publish_count / elapsed
            self.get_logger().info(
                f"TF actual rate: {actual_rate:.1f} Hz "
                f"(target: {self.tf_rate} Hz, published {self.tf_publish_count} times)"
            )
    
    # ================= SERIAL =================
    def connect_serial(self, port, baud):
        try:
            self.serial = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT)
            time.sleep(2)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            self.connected = True
            self.running = True
            self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
            self.read_thread.start()
            
            self.send_command("GET_ENCODERS")
            self.get_logger().info(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.connected = False
    
    def send_command(self, cmd):
        if self.connected and self.serial:
            try:
                self.serial.write(f"{cmd}\n".encode())
                return True
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")
        return False
    
    def read_loop(self):
        while self.running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self.parse_response(line)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
            time.sleep(0.001)
    
    def parse_response(self, line):
        if self.shutting_down:
            return
            
        if line.startswith("ODOM:"):
            try:
                parts = line[5:].split(',')
                if len(parts) >= 8:
                    with self.lock:
                        self.x = float(parts[0])
                        self.y = float(parts[1])
                        self.theta = float(parts[2])
                        self.v_linear = float(parts[3])
                        self.v_angular = float(parts[4])
                        self.left_ticks = int(float(parts[6]))
                        self.right_ticks = int(float(parts[7]))
                        self.last_odom_update = time.time()
                        self.odom_stale_warned = False
                    self.publish_odometry_msg()
            except Exception as e:
                if not self.shutting_down:
                    self.get_logger().error(f"Odom parse error: {e}", throttle_duration_sec=5.0)
                    
        elif line.startswith("IMU:"):
            try:
                parts = line[4:].split(',')
                if len(parts) >= 6:
                    ax, ay, az = float(parts[0]), float(parts[1]), float(parts[2])
                    gx, gy, gz = float(parts[3]), float(parts[4]), float(parts[5])
                    self.publish_imu_raw(ax, ay, az, gx, gy, gz)
            except Exception as e:
                self.get_logger().error(f"IMU parse error: {e}")
                
        elif line.startswith("========") or line.startswith("READY"):
            self.get_logger().info(f"Arduino: {line}")
        elif line.startswith("ERROR:"):
            self.get_logger().error(f"Arduino: {line}")
    
    def cmd_vel_callback(self, msg):
        linear = max(min(msg.linear.x, MAX_LINEAR), -MAX_LINEAR)
        angular = max(min(msg.angular.z, MAX_ANGULAR), -MAX_ANGULAR)
        with self.lock:
            self.target_v = linear
            self.target_w = angular
            self.last_cmd_time = time.time()
    
    def cmd_loop(self):
        with self.lock:
            v = self.target_v
            w = self.target_w
            time_since_cmd = time.time() - self.last_cmd_time
        
        if time_since_cmd > 1.0 and (v != 0.0 or w != 0.0):
            with self.lock:
                self.target_v = 0.0
                self.target_w = 0.0
            v = 0.0
            w = 0.0
        
        self.send_command(f"CMD_VEL:{v:.3f},{w:.3f}")
    
    def tf_loop(self):
        """TF publishing loop - should run at 50Hz"""
        if self.shutting_down:
            return
        
        # DEBUG: Count publishes
        self.tf_publish_count += 1
            
        with self.lock:
            x = self.x
            y = self.y
            theta = self.theta
            odom_age = time.time() - self.last_odom_update
        
        if odom_age > 0.5 and not self.odom_stale_warned:
            self.get_logger().warn(
                f"Odometry data is {odom_age:.1f}s old",
                throttle_duration_sec=5.0
            )
            self.odom_stale_warned = True
        
        current_time = self.get_clock().now().to_msg()
        q = quaternion_from_euler(0, 0, theta)
        
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odometry_msg(self):
        if self.shutting_down:
            return
            
        with self.lock:
            x, y, theta = self.x, self.y, self.theta
            v_lin, v_ang = self.v_linear, self.v_angular
        
        current_time = self.get_clock().now().to_msg()
        q = quaternion_from_euler(0, 0, theta)
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = v_lin
        odom.twist.twist.angular.z = v_ang
        
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.01
        
        self.odom_pub.publish(odom)
    
    def publish_imu_raw(self, ax, ay, az, gx, gy, gz):
        if self.shutting_down:
            return
            
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        imu_msg.orientation_covariance[0] = -1
        
        imu_msg.angular_velocity.x = math.radians(gx / 131.0)
        imu_msg.angular_velocity.y = math.radians(gy / 131.0)
        imu_msg.angular_velocity.z = math.radians(gz / 131.0)
        
        imu_msg.linear_acceleration.x = (ax / 16384.0) * 9.81
        imu_msg.linear_acceleration.y = (ay / 16384.0) * 9.81
        imu_msg.linear_acceleration.z = (az / 16384.0) * 9.81
        
        self.imu_pub.publish(imu_msg)
    
    def destroy_node(self):
        self.get_logger().info("Shutting down...")
        self.shutting_down = True
        self.running = False
        
        if self.serial and self.serial.is_open:
            try:
                self.send_command("STOP")
                time.sleep(0.1)
            except:
                pass
        
        if hasattr(self, 'read_thread') and self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()