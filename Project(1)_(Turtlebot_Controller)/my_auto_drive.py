import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from PySide6.QtCore import QObject, Signal

class DrivingSignals(QObject):
    Driving_status = Signal(bool)

class AutoDrive(Node):
  def __init__(self):
    super().__init__('auto_drive')
    self.qos_profile = QoSProfile(depth = 10)
    self.scan_sub = self.create_subscription(
      LaserScan,
      '/scan',
      self.scan_callback,
      qos_profile=qos_profile_sensor_data
    )
    self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, self.qos_profile
    )
    self.move_turtle = self.create_publisher(Twist, '/cmd_vel', self.qos_profile)
    self.state_pub = self.create_publisher(String, "/driving_state", self.qos_profile)

    self.timer = self.create_timer(0.1, self.turtle_move)
    self.timer.cancel()

    self.odom = Odometry()
    self.twist = Twist()

    self.velocity = 0.2
    self.angular = 13 * (90.0 / 180.0) * math.pi / 100.0
    self.scan_ranges = []
    self.has_scan_received = False
    self.left_min = 0
    self.right_min = 0
    self.front_min = 0
    self.before_position_x = Odometry().pose.pose.position.x
    self.before_position_y = Odometry().pose.pose.position.y
    self.last_record_time = time.time()
    self.adjust_pose = False
    self.set_adjust_time = 5.0
    self.signals = DrivingSignals()

  def odom_callback(self, msg):
        self.odom = msg

  def init_twist(self):
        self.before_position_x = Odometry().pose.pose.position.x
        self.before_position_y = Odometry().pose.pose.position.y
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.move_turtle.publish(self.twist)
        self.get_logger().info(f'Published mesage: {self.twist.linear}, {self.twist.angular}')

  def turn(self, direction):
    if direction == "L":
      self.twist.linear.x = 0.0
      self.twist.angular.z = self.angular
    else:
      self.twist.linear.x = 0.0
      self.twist.angular.z = -self.angular
    self.move_turtle.publish(self.twist)
    self.get_logger().info(f'Published mesage: {self.twist.linear}, {self.twist.angular}')

  def turn_for_adjust(self):
    # print("turn_for_adjustment")
    if self.front_min >= 1.5:
        self.adjust_pose = False
        self.init_twist()
        return

    self.twist.linear.x = 0.0
    self.twist.angular.z = self.angular
    self.move_turtle.publish(self.twist)

    self.get_logger().info(f'Published mesage: {self.twist.linear}, {self.twist.angular}')

  def turtle_move(self):
    state_msg = String()

    if self.adjust_pose:
       state_msg.data = "Turning for Adjustment"
       self.turn_for_adjust()
    else:
      current_time = time.time()
      current_position_x = self.odom.pose.pose.position.x
      current_position_y = self.odom.pose.pose.position.y

      if current_time-self.last_record_time >= self.set_adjust_time:
        diff_x = abs(current_position_x - self.before_position_x)
        diff_y = abs(current_position_y - self.before_position_y)
        if diff_x < 0.01 and diff_y < 0.01:
            # print("adjust position!")
            self.adjust_pose = True
        self.before_position_x = current_position_x
        self.before_position_y = current_position_y
        self.last_record_time = current_time

      if not self.has_scan_received:
        self.init_twist()
      elif self.front_min < 1.5:
        if self.left_min < self.right_min:
          state_msg.data = "Turn Right"
          self.turn("R")
        else:
          state_msg.data = "Turn Left"
          self.turn("L")
      else:
        state_msg.data = "Move Straight"
        self.twist.linear.x = self.velocity
        self.twist.angular.z = 0.0
        self.move_turtle.publish(self.twist)
        self.get_logger().info(f'Published mesage: {self.twist.linear}, {self.twist.angular}')

    self.state_pub.publish(state_msg)

  def scan_callback(self, msg):
    self.scan_ranges = msg.ranges
    self.has_scan_received = True
    scan_range = len(self.scan_ranges)-1
    frontL_range = int(scan_range / 12)
    frontR_range = int(scan_range * 11 / 12)
    left_range = int(scan_range / 4)
    right_range = int(scan_range * 3 / 4)
    frontL_min = min(self.scan_ranges[0:frontL_range])
    frontR_min = min(self.scan_ranges[frontR_range:scan_range])
    self.front_min = min(frontL_min, frontR_min)
    self.left_min = min(self.scan_ranges[0:left_range])
    self.right_min = min(self.scan_ranges[right_range:scan_range])
    self.get_logger().info(f'left_min:{self.left_min},right_min: {self.right_min}', throttle_duration_sec=2)

  def start_drive(self):
    if self.timer.is_canceled():
        self.timer.reset()
        self.signals.Driving_status.emit(True)
        self.get_logger().info('AutoDrive Started.')

  def stop_drive(self):
    if not self.timer.is_canceled():
        self.timer.cancel()
        self.init_twist() # 물리적으로 로봇 멈춤
        self.signals.Driving_status.emit(False)
        self.get_logger().info('AutoDrive Stopped.')

  def output_state(self):
     return

def main(args=None):
  rclpy.init(args=args)
  node = AutoDrive()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard interrupt!!!!')
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main':
	  main()
