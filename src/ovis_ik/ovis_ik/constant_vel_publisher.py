import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ConstantVelPublisher(Node):
    def __init__(self):
        super().__init__('constant_vel_publisher')
        self.pub = self.create_publisher(Twist, '/mouse_twist', 10)
        self.timer = self.create_timer(0.01, self.publish_msg)

    def publish_msg(self):
        msg = Twist()
        msg.linear.z = 1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConstantVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
