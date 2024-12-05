import rclpy
import sys
from rclpy.node import Node

from car_interface.msg import Car
import time


class CarPublisher(Node):
    def __init__(self):
        super().__init__('car_publisher')
        self.car_info_publisher_ = self.create_publisher(Car, '/start_car', 10)
        self.test_car = sys.argv[1]
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Car()
        if self.test_car == 'PR001' or self.test_car == 'PR002':
            msg.car = self.test_car
            self.car_info_publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.car)
        else:
            self.get_logger().error('"%s": 해당 차량이 존재하지 않습니다.' % self.test_car)
        return


def main(args=None):
    rclpy.init(args=args)
    car_publisher = CarPublisher()
    rclpy.spin(car_publisher)
    car_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()