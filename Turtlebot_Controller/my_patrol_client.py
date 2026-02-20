import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebot3_msgs.action import Patrol
from rclpy.callback_groups import ReentrantCallbackGroup
from PySide6.QtCore import QObject, Signal

class ClientSignals(QObject):
    patrol_status = Signal(bool)

class Turtlebot3PatrolClient(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_client')

        print('TurtleBot3 Patrol Client')
        print('----------------------------------------------')
        print('Input below')
        print('mode: s: square, t: triangle')
        print('travel_distance (unit: m)')
        print('patrol_count')
        print('----------------------------------------------')

        self._action_client = ActionClient(self, Patrol, 'turtlebot3', callback_group=ReentrantCallbackGroup())

        self.mode = 1.0
        self.travel_distance = 1.0
        self.patrol_count = 1
        self.signals = ClientSignals()

    def send_goal(self, mode, travel_distance, patrol_count):
        print("send_goal")
        self.signals.patrol_status.emit(True)

        self.mode = float(mode)
        self.travel_distance = float(travel_distance)
        self.patrol_count = int(patrol_count)

        goal_msg = Patrol.Goal()
        goal_msg.goal.x = float(self.mode)
        goal_msg.goal.y = float(self.travel_distance)
        goal_msg.goal.z = float(self.patrol_count)

        if not self._action_client.server_is_ready():
            self.get_logger().info('Action server not ready, waiting...')
            if not self._action_client.wait_for_server(timeout_sec=0.5):
                self.get_logger().error('Server still not available')
                return

        self._send_goal_future = \
            self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        print("goal_response_callback")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.signals.patrol_status.emit(False)
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        self.signals.patrol_status.emit(False)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))


def main(args=None):
    rclpy.init(args=args)

    turtlebot3_patrol_client = Turtlebot3PatrolClient()

    rclpy.spin(turtlebot3_patrol_client)


if __name__ == '__main__':
    main()
