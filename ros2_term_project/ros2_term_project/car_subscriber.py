import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


class CarSubscriber(Node):

    def __init__(self):
        super().__init__('car_subscriber')


        self.subscription = self.create_subscription(
            String,
            'start_car',
            self.listener_callback,
            10)
        self.subscription

        self.twist_publisher_ = None

        # 회전 여부
        self.turn = False
        # 장애물 여부
        self.obstacle_found = False
        # 정지 여부
        self.stop = False
        # 정지선 종류 구분
        self.count = 0


    def listener_callback(self, msg):
        car = msg.data
        self.get_logger().info('I heard: "%s"' % msg.data)
        time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    car_subscriber = CarSubscriber()
    rclpy.spin(car_subscriber)
    car_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()