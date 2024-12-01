import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .line_tracker import LineTracker
import cv_bridge


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker):
        super().__init__('line_follower')
        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

        # 차량에 전달할 속도 정보를 전달하는 publisher
        self.twist_info_publisher_ = self.create_publisher(
            Twist,
            'twist_info',
            10
        )

        # 주행 상태 정보를 전달할 publisher
        self.drive_issue_publisher_ = self.create_publisher(
            String,
            'drive_issue',
            10
        )

        self.invasion_info_publisher_ = self.create_publisher(
            String,
            'invasion_info',
            10
        )
        self.image_subscription_ = None


        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        self._subscription = self.create_subscription(Image, '/PR001_camera/image_raw',
                                                            self.lane_image_callback, 10)

    def timer_callback(self):
        msg = String()
        msg.data = str(self.line_tracker._invasion)

        # 차선 침범 횟수 전달
        self.invasion_info_publisher_.publish(msg)

    def lane_image_callback(self, image: Image):
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.line_tracker.process(img)

        # 회전 속도 조절
        twist = Twist()
        msg = String()
        twist.angular.z = (-1) * self.line_tracker._delta / 110

        # 방향 조정 최대치 설정
        if twist.angular.z > 0.7:
            twist.angular.z = 0.7

        # 회전 인식
        if twist.angular.z > 0.08:
            msg.data = '회전'
        else:
            msg.data = '직진'

        # 회전 시 감속
        if twist.angular.z > 0.3:
            twist.linear.x = 3.0
        else:
            twist.linear.x = 6.0

        self.drive_issue_publisher_.publish(msg)
        self.twist_info_publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    tracker = LineTracker()
    follower = LineFollower(tracker)
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
