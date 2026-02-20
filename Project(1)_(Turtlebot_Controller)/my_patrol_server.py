import math
import threading
import time

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from turtlebot3_msgs.action import Patrol


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_server')

        print('TurtleBot3 Patrol Server')
        print('----------------------------------------------')

        self._action_server = ActionServer(
            self,
            Patrol,
            'turtlebot3',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback)

        self.goal_msg = Patrol.Goal()
        self.twist = Twist()
        self.odom = Odometry()
        self.position = Point()
        self.rotation = 0.0

        self.front_min = 0
        self.has_scan_received = False
        self.stop_sign = False
        self.detect_obs = False

        self.linear_x = 1.0
        self.angular_z = 4.0

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

    def set_stop(self):
        self.stop_sign = True

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True
        scan_range = len(self.scan_ranges)-1
        frontL_range = int(scan_range / 12)
        frontR_range = int(scan_range * 11 / 12)
        frontL_min = min(self.scan_ranges[0:frontL_range])
        frontR_min = min(self.scan_ranges[frontR_range:scan_range])
        self.front_min = min(frontL_min, frontR_min)
        if self.front_min < 1.5 or self.stop_sign:
            self.detect_obs = True
        else:
            self.detect_obs = False
        self.get_logger().info(f'front_min:{self.front_min} has_scan_received:{self.has_scan_received}', throttle_duration_sec=2)

    def init_twist(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def odom_callback(self, msg):
        self.odom = msg

    def get_yaw(self):
        q = self.odom.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def go_front(self, position, length):
        while True:
            if self.detect_obs:
                break
            position += self.twist.linear.x
            if position >= length:
                break
            self.twist.linear.x = self.linear_x
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            time.sleep(1)
        self.init_twist()

    def turn(self, target_angle):
        initial_yaw = self.get_yaw()
        target_yaw = initial_yaw + (target_angle * math.pi / 180.0)

        while True:
            # rclpy.spin_once(self, timeout_sec=0.1)

            if self.detect_obs:
                break

            current_yaw = self.get_yaw()
            yaw_diff = abs(
                math.atan2(
                    math.sin(target_yaw - current_yaw),
                    math.cos(target_yaw - current_yaw)
                )
            )

            if yaw_diff < 0.01:
                break

            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_z
            self.cmd_vel_pub.publish(self.twist)

        self.init_twist()

    def goal_callback(self, goal_request):
        print("goal_callback")

        self.goal_msg = goal_request

        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        print("execyte_callback")
        self.get_logger().info('Executing goal...')
        feedback_msg = Patrol.Feedback()

        length = self.goal_msg.goal.y
        iteration = int(self.goal_msg.goal.z)

        if self.goal_msg.goal.x == 1:
            for count in range(iteration):
                if self.detect_obs:
                    break
                else:
                    self.square(feedback_msg, goal_handle, length)
            if self.detect_obs:
                feedback_msg.state = 'square patrol not complete!! (detect obstacle)'
            else:
                feedback_msg.state = 'square patrol complete!!'
        elif self.goal_msg.goal.x == 2:
            for count in range(iteration):
                if self.detect_obs:
                    break
                else:
                    self.triangle(feedback_msg, goal_handle, length)
            if self.detect_obs:
                feedback_msg.state = 'triangle patrol not complete!! (detect obstacle)'
            else:
                feedback_msg.state = 'triangle patrol complete!!'

        goal_handle.succeed()
        result = Patrol.Result()
        result.result = feedback_msg.state

        self.init_twist()
        self.get_logger().info('Patrol complete.')
        self.stop_sign = False
        return result

    def square(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 13 * (90.0 / 180.0) * math.pi / 100.0

        for i in range(4):
            self.position.x = 0.0
            self.angle = 0.0

            if self.detect_obs:
                break

            self.go_front(self.position.x, length)
            self.turn(90.0)

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        self.init_twist()

    def triangle(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 8 * (120.0 / 180.0) * math.pi / 100.0

        for i in range(3):
            self.position.x = 0.0
            self.angle = 0.0

            if self.detect_obs:
                break

            self.go_front(self.position.x, length)
            self.turn(120.0)

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        self.init_twist()


def main(args=None):
    rclpy.init(args=args)

    turtlebot3_patrol_server = Turtlebot3PatrolServer()

    rclpy.spin(turtlebot3_patrol_server)


if __name__ == '__main__':
    main()
