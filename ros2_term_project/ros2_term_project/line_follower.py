from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from car_interface.msg import Car
import cv_bridge
import datetime as dt
from .line_tracker import LineTracker
import time


class LineFollower(Node):
    def __init__(self, car_name: str, line_tracker: LineTracker):
        super().__init__('line_follower')

        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()

        # 카메라 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )

        # /scan 토픽 구독자 생성
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
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
        self.twist.linear.x = 5.0  # 전진 속도
        self.original_linear_speed = self.twist.linear.x  # 원래 속도 저장
        self.rotation_threshold = 0.15  # 회전 속도 기준 (angular.z 값의 절대값)
        self.max_angular_speed = 4.0  # 최대 회전 속도 제한
        self.img = None

        # 장애물 회피 관련 변수 추가
        self.obstacle_found = False  # 장애물 발견 여부
        self.waiting_start_time = None  # 대기 시작 시간
        self.avoidance_state = LineFollower.State.WAITING  # 회피 상태
        self.avoidance_sign = 1  # 회피 방향
        self.avoidance_start_delta = 0  # 회피 시작 시간 차이

        # 1초마다 메시지를 발행하도록 타이머 설정
        self.timer = self.create_timer(1.0, self.publish_twist)

    def publish_twist(self):
        # Twist 메시지 발행
        self.get_logger().info(f'Publishing Twist: linear.x={self.twist.linear.x}, angular.z={self.twist.angular.z}')
        self.twist_publisher.publish(self.twist)

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.line_tracker.process(img)

        # 회전 속도 계산
        angular_velocity = -self.line_tracker.delta / 150  # 비율 조정하여 더 적극적으로 회전 반영
        self.twist.angular.z = angular_velocity

        # 회전 속도의 최대값 제한
        if self.twist.angular.z > self.max_angular_speed:
            self.twist.angular.z = self.max_angular_speed
        elif self.twist.angular.z < -self.max_angular_speed:
            self.twist.angular.z = -self.max_angular_speed

        # 회전 중일 때 속도 감소
        if abs(self.twist.angular.z) > self.rotation_threshold:
            self.twist.linear.x = 3.0  # 회전 중 속도를 줄이되 최소 3.0m/s 유지
        else:
            self.twist.linear.x = self.original_linear_speed  # 회전 끝났으면 원래 속도로 복귀

        if self.obstacle_found: return

        # Twist 명령 발행
        self.get_logger().info(f'linear.x = {self.twist.linear.x}, angular.z = {self.twist.angular.z}')
        self.twist_publisher.publish(self.twist)

    def scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)  # 레이저 데이터에서 최소 거리 계산
        if not self.obstacle_found and min_distance < 7.0:  # 7m 이내의 장애물 발견 시
            self.stop()  # 정지
            self.obstacle_found = True  # 장애물 발견 플래그 설정
            if self.waiting_start_time is None:  # 대기 시작 시간이 설정되지 않았으면
                # 현재 시간을 대기 시작 시간으로 저장
                self.waiting_start_time = dt.datetime.now()
            self.get_logger().info(
                'An obstacle found in %.2f m' % min(msg.ranges)
            )  # 로그 출력: 발견된 장애물 거리

        if self.obstacle_found:
            if self.waiting_start_time is None: return
            # 의도적으로 4.0 + 1.0 기준 설정
            if self.avoidance_state == LineFollower.State.WAITING and min_distance > 7.0:
                self.obstacle_found = False
                self.waiting_start_time = None
                self.go_straight()

    def go_straight(self):
        self.twist.linear.x = 0.8
        self.twist.angular.z = (-1) * self.avoidance_sign * 0.15 \
            if abs(self.avoidance_start_delta) > 25 else (-1) * self.avoidance_sign * 0.3
        self.twist_publisher.publish(self.twist)

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist)
        self.get_logger().info("Vehicle stopped.")

    @property
    def publisher(self):
        return self._publisher

    class State(Enum):
        WAITING = 0
        STEP_ASIDE = 1
        GO_STRAIGHT = 2
        STEP_IN = 3

def main():
    rclpy.init()

    # 차량 이름 가져오기 (기본값: PR001)
    car_name = 'PR001'

    tracker = LineTracker()
    follower = LineFollower(car_name, tracker)

    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        follower.stop()
        print("Stopped by user")

if __name__ == '__main__':
    main()
