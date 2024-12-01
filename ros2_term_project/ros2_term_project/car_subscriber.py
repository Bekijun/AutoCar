import rclpy
from rclpy.node import Node
from car_interface.msg import Car
from geometry_msgs.msg import Twist
import time


class CarSubscriber(Node):

    def __init__(self):
        super().__init__('car_subscriber')

        self.subscription_ = self.create_subscription(
            Car,
            'start_car',
            self.car_info_listener_callback,
            10)
        self.subscription_

        self.twist_publisher_ = None

        # 회전 여부
        self.turn = False
        # 장애물 여부
        self.obstacle_found = False
        # 정지 여부
        self.stop = False
        # 정지선 종류 구분
        self.count = 0

    def car_info_listener_callback(self, msg: Car):
        # 지정 차량
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)

        time.sleep(5)

        # 차량에 속도 정보를 전달할 publisher
        self.twist_publisher_ = self.create_publisher(Twist, '/demo/' + car + '_cmd_demo', 10)

        # 차량 출발
        twist = Twist()
        for i in range(200):
            twist.linear.x = 6.0
            self.twist_publisher_.publish(twist)
            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    car_subscriber = CarSubscriber()
    rclpy.spin(car_subscriber)
    car_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()