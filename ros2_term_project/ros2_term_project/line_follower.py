import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from car_interface.msg import Car
import cv_bridge
from .line_tracker import LineTracker
from .stop_line_tracker import StopLineTracker
import time

class LineFollower(Node):
    def __init__(self, car_name: str, line_tracker: LineTracker, stop_line_tracker: StopLineTracker):
        super().__init__('line_follower')

        self.line_tracker = line_tracker
        self.stop_line_tracker = stop_line_tracker
        self.bridge = cv_bridge.CvBridge()

        # 카메라 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )

        # 차량 이름 설정
        self.car_name = car_name

        # Twist 메시지를 발행할 Publisher 생성
        self.twist_publisher = self.create_publisher(
            Twist, f'/demo/{self.car_name}_cmd_demo', 10
        )

        # 기본 Twist 메시지 생성
        self.twist = Twist()
        self.twist.linear.x = 10.0  # 전진 속도
        self.twist.angular.z = 0.0  # 회전 없음
        self.max_angular_speed = 2.0  # 최대 회전 속도를 더 높임

        # 1초마다 메시지를 발행하도록 타이머 설정
        self.timer = self.create_timer(1.0, self.publish_twist)

        # 정지선 관련 상태 플래그
        self.stop_detected = False
        self.stop_time = None

        # 상태 플래그
        self.turning = False
        self.post_turn_straight = False
        self.post_turn_count = 0
        self.turn_threshold = 300

    def publish_twist(self):
        # Twist 메시지 발행
        self.get_logger().info(f'Publishing Twist: linear.x={self.twist.linear.x}, angular.z={self.twist.angular.z}')
        self.twist_publisher.publish(self.twist)

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.line_tracker.process(img)
        self.stop_line_tracker.process(img)

        # 정지선 인식 처리
        if self.stop_line_tracker.is_stop_detected() and not self.stop_detected:
            self.stop_detected = True
            self.stop_time = time.time()
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.twist_publisher.publish(self.twist)
            self.get_logger().info("Stop line detected, stopping vehicle.")

        # 정지 후 3초 대기 후 다시 출발
        if self.stop_detected:
            if time.time() - self.stop_time >= 3:
                self.stop_detected = False
                self.twist.linear.x = 10.0
                self.twist.angular.z = 0.0
                self.get_logger().info("Resuming movement.")
                print("Resuming movement")
            else:
                self.twist.linear.x = 0.0
                return

        if not self.stop_detected:
            # delta 값을 기반으로 각속도 설정
            delta = self.line_tracker.delta

            # 각속도 조정: delta를 더욱 강조하여 더 빠르게 회전하도록 설정
            angular_z = -delta / 245.0

            # 각속도는 최대 각속도를 넘지 않도록 제한
            if angular_z > self.max_angular_speed:
                angular_z = self.max_angular_speed
            elif angular_z < -self.max_angular_speed:
                angular_z = -self.max_angular_speed

            # 회전 조건 확인
            if abs(delta) > self.turn_threshold:
                self.turning = True

            # 회전 중 로직
            if self.turning:
                self.twist.linear.x = 1.2
                self.twist.angular.z = angular_z

                if abs(delta) < 150:
                    self.turning = False
                    self.post_turn_straight = True
                    self.post_turn_count = 0
            elif self.post_turn_straight:
                self.post_turn_count += 1
                self.twist.linear.x = 5.0
                self.twist.angular.z = 0.0

                if self.post_turn_count > 30:
                    self.post_turn_straight = False
            else:
                self.twist.linear.x = 10.0
                self.twist.angular.z = angular_z

            # Twist 명령 발행
            self.twist_publisher.publish(self.twist)

    def stop(self):
        # 차량 정지
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist)
        self.get_logger().info("Vehicle stopped.")

def main():
    rclpy.init()

    # 차량 이름 가져오기 (기본값: PR001)
    car_name = 'PR001'

    tracker = LineTracker()
    stop_tracker = StopLineTracker()
    follower = LineFollower(car_name, tracker, stop_tracker)

    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        follower.stop()
        print("Stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
