import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class TurtleYOLONode(Node):
    def __init__(self):
        super().__init__('turtle_yolo_node')

        # 확인한 경로 사용
        self.model_path = '/home/pg7728/robot_ws/src/2nd_project_pkg/2nd_project_pkg/yolov8n.pt'
        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()

        # 터틀봇 카메라 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 토픽 이름
            self.image_callback,
            10)
        self.get_logger().info('YOLO 인식 노드가 시작되었습니다!')

    def image_callback(self, msg):
        # ROS 영상을 OpenCV 형식으로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLO 진행
        results = self.model(frame, stream=True)

        for r in results:
            annotated_frame = r.plot()  # 인식 결과(주석이 붙은)가 그려진 프레임

            # 화면 출력
            cv2.imshow("SF_ONE TurtleBot - YOLO", annotated_frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleYOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
