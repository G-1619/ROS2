import sys
import rclpy
import math
import threading
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from PySide6.QtCore import QFile, QThread, Signal, Slot
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from turtlebot3_msgs.action import Patrol
# ros2 의 module 참조방식
from turtlebot_controller_ui import Ui_Form
from my_patrol_server import Turtlebot3PatrolServer
from my_patrol_client import Turtlebot3PatrolClient
from my_auto_drive import AutoDrive

# GUI 의 쓰레드와 충돌을 피하기위해 qthread를 상속받아서 rclpy 쓰레드를 구성
class RclpyThread(QThread):
    def __init__(self, executor):
        super().__init__()
        self.executor = executor

    def run(self):
        try:
            self.executor.spin()
        finally:
            rclpy.shutdown()

class MyTurtlebot3Controller(QWidget):
    def __init__(self):
        # ui에 관한 코드
        super(MyTurtlebot3Controller, self).__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.ui.stackedWidget.setCurrentIndex(0)

        # rclpy 쓰레드를 실행
        rclpy.init()
        qos_profile = QoSProfile(depth=10)
        self.velocity = 0.0
        self.rotation = 0.0

        # button publisher Page 정의
        self.btn_pub_node = Node("Button_publisher")
        self.Button_publisher = self.btn_pub_node.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.ui.go_BP.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.btn_Go.clicked.connect(self.btn_pub_Go_clicked)
        self.ui.btn_Back.clicked.connect(self.btn_pub_Back_clicked)
        self.ui.btn_TRight.clicked.connect(self.btn_pub_TRight_clicked)
        self.ui.btn_TLeft.clicked.connect(self.btn_pub_TLeft_clicked)
        self.ui.btn_Stop.clicked.connect(self.btn_pub_Stop_clicked)
        self.ui.btn_MP.clicked.connect(self.btn_MP_clicked)

        # Figure Server Page 정의
        self.patrol_server_node = Turtlebot3PatrolServer()
        self.patrol_client_node = Turtlebot3PatrolClient()
        self.ui.go_FP.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(2))
        self.distance = [1, 2, 3, 4, 5]
        self.move_count = [1, 2, 3, 4, 5]
        self.ui.box_distance.addItems(list(map(str,self.distance)))
        self.ui.box_count.addItems(list(map(str,self.move_count)))
        self.ui.box_distance.setCurrentIndex(0)
        self.ui.box_count.setCurrentIndex(0)
        self.ui.btn_triangle.setChecked(True)
        self.ui.btn_square.setChecked(False)
        self.ui.btn_startFigure.clicked.connect(self.btn_startFigure_clicked)
        self.ui.btn_stopFigure.clicked.connect(self.btn_stopFigure_clicked)
        self.ui.btn_stopFigure.setEnabled(False)
        self.ui.btn_MP_2.clicked.connect(self.btn_MP_clicked_inFP)
        self.patrol_client_node.signals.patrol_status.connect(self.update_FP_button)

        # 충돌회피 Auto Drive Page 정의
        self.auto_pub_node = AutoDrive()
        self.auto_sub_node = Node("Driving_state_Subscriber")
        self.auto_sub = self.auto_sub_node.create_subscription(String, '/driving_state', self.state_callback, qos_profile)
        self.ui.go_AP.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(3))
        self.ui.btn_autoStart.clicked.connect(self.btn_autoStart_clicked)
        self.ui.btn_autoStop.clicked.connect(self.btn_autoStop_clicked)
        self.ui.btn_MP_3.clicked.connect(self.btn_MP_clicked_inAP)
        self.last_state_msg = String()
        self.last_state_msg.data = "Stop"

        # MultiThread 설정
        self.executor = MultiThreadedExecutor(num_threads=12)

        # node 추가.
        self.executor.add_node(self.btn_pub_node)
        self.executor.add_node(self.patrol_server_node)
        self.executor.add_node(self.patrol_client_node)
        self.executor.add_node(self.auto_pub_node)
        self.executor.add_node(self.auto_sub_node)

        # rclpy Thread 시작.
        self.rclpy_thread = RclpyThread(self.executor)
        self.rclpy_thread.start()

    def update_FP_button(self, is_running):
        self.ui.btn_startFigure.setEnabled(not is_running)
        self.ui.btn_stopFigure.setEnabled(is_running)
        self.ui.box_distance.setEnabled(not is_running)
        self.ui.box_count.setEnabled(not is_running)
        self.ui.btn_triangle.setEnabled(not is_running)
        self.ui.btn_square.setEnabled(not is_running)

    def btn_MP_clicked_inFP(self):
        self.patrol_server_node.set_stop()
        self.velocity = 0.0
        self.rotation = 0.0
        self.pub_msg()
        self.ui.stackedWidget.setCurrentIndex(0)

    def btn_MP_clicked(self):
        #임시
        self.velocity = 0.0
        self.rotation = 0.0
        self.pub_msg()
        self.ui.stackedWidget.setCurrentIndex(0)

    def btn_MP_clicked_inAP(self):
        self.auto_pub_node.stop_drive()
        self.ui.list_Dlog.clear()
        self.last_state_msg.data = "Stop"
        self.ui.stackedWidget.setCurrentIndex(0)

    def btn_autoStop_clicked(self):
        self.auto_pub_node.stop_drive()
        self.last_state_msg.data = "Stop"
        self.ui.list_Dlog.addItem(f"Turtlebot Status: {self.last_state_msg.data}.")

    def btn_autoStart_clicked(self):
        self.ui.list_Dlog.clear()
        self.last_state_msg.data = "Stop"
        self.auto_pub_node.start_drive()

    def state_callback(self, msg):
        if msg.data != self.last_state_msg.data:
            self.last_state_msg.data = msg.data
            self.ui.list_Dlog.addItem(f"Turtlebot Status: {self.last_state_msg.data}.")

    def btn_stopFigure_clicked(self):
        self.patrol_server_node.set_stop()
        self.velocity = 0.0
        self.rotation = 0.0
        self.pub_msg()

    def btn_startFigure_clicked(self):
        if self.ui.btn_triangle:
            mode = 2
        else:
            mode = 1
        travel_distance = self.distance[self.ui.box_distance.currentIndex()]
        patrol_count = self.move_count[self.ui.box_count.currentIndex()]
        self.patrol_client_node.send_goal(mode, travel_distance, patrol_count)

    def pub_msg(self):
        msg = Twist()
        msg.linear.x = self.velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.rotation
        self.Button_publisher.publish(msg)
        self.btn_pub_node.get_logger().info(f'Published mesage: {msg.linear}, {msg.angular}')

    def btn_pub_Go_clicked(self):
        self.velocity = 0.5
        self.rotation = 0.0
        self.pub_msg()

    def btn_pub_Back_clicked(self):
        self.velocity = -0.5
        self.rotation = 0.0
        self.pub_msg()

    def btn_pub_TRight_clicked(self):
        self.velocity = 0.0
        self.rotation = -0.3
        self.pub_msg()

    def btn_pub_TLeft_clicked(self):
        self.velocity = 0.0
        self.rotation = 0.3
        self.pub_msg()

    def btn_pub_Stop_clicked(self):
        self.velocity = 0.0
        self.rotation = 0.0
        self.pub_msg()

    def closeEvent(self, event):
        # 종료 시 리소스 정리
        print("쓰레드 및 노드 종료")
        self.executor.shutdown()
        self.rclpy_thread.quit()
        self.rclpy_thread.wait()
        self.btn_pub_node.destroy_node()
        self.patrol_server_node.destroy_node()
        self.patrol_client_node.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)

def main(args=None):
    app = QApplication(sys.argv)
    window = MyTurtlebot3Controller()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()








