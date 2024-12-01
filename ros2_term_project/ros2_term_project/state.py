import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class State(Node):
    def __init__(self):
        super().__init__('state')

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량이 지정되면 속도 정보를 받는 subscription 생성
        self.vel_info_subcription = self.create_subscription(
            Twist,
            '/demo/' + car + '_cmd_demo',
            self.vel_info_listener_callback,
            10
        )

    def vel_info_listener_callback(self, msg: Twist):
        self.get_logger().info('Received velocity info: %s' % msg.linear.x)

def main(args=None):
    rclpy.init(args=args)

    state = State()

    rclpy.spin(state)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()