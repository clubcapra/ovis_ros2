import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from roboticstoolbox import DHRobot, RevoluteDH
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class VelocityNode(Node):
    def __init__(self):
        super().__init__('velocity_node')

        self.twist_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.twist_callback,
            10,
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10,
        )
        self.vel_pub = self.create_publisher(Float64MultiArray, '/joint_vel', 10)

        # Joint naming and axis ordering are aligned with ovis_description URDF.
        # Joints: ovis_joint_1..6
        # Axes :  z, y, -y, -x, y, -x
        self.joint_names = [
            'ovis_joint_1',
            'ovis_joint_2',
            'ovis_joint_3',
            'ovis_joint_4',
            'ovis_joint_5',
            'ovis_joint_6',
        ]

        # DH model tuned with URDF-scale distances (meters).
        self.robot = DHRobot(
            [
                RevoluteDH(d=0.10001, a=0.0, alpha=np.pi / 2),
                RevoluteDH(d=0.0, a=-0.30039, alpha=np.pi),
                RevoluteDH(d=0.0, a=0.0, alpha=-np.pi / 2, offset=-np.pi / 2),
                RevoluteDH(d=0.33480, a=0.0, alpha=-np.pi / 2),
                RevoluteDH(d=0.0, a=0.0, alpha=np.pi / 2),
                RevoluteDH(d=0.33, a=0.0, alpha=0.0),
            ],
            name='ovis',
        )

        self.q = np.zeros(6)
        self.latest_twist = np.zeros(6)

    def joint_callback(self, msg: JointState):
        if not msg.position:
            return

        index = {name: i for i, name in enumerate(msg.name)}
        try:
            self.q = np.array([msg.position[index[name]] for name in self.joint_names])
        except KeyError:
            # Fallback if names are not available in expected order.
            if len(msg.position) >= 6:
                self.q = np.array(msg.position[:6])

    def twist_callback(self, msg: TwistStamped):
        self.latest_twist = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        ])
        self.compute_and_publish()

    def compute_and_publish(self):
        jacobian = self.robot.jacobe(self.q)
        q_dot = np.linalg.pinv(jacobian) @ self.latest_twist

        q_dot = q_dot[:6] * [1.0,1.0,1.0,1.0,1.0,1.0]

        if np.any(np.abs(q_dot) > 4.0):
            self.get_logger().warning('Joint velocity exceeded limit; command dropped.')
            q_dot = np.zeros_like(q_dot)

        out_msg = Float64MultiArray()
        # out_msg.header.stamp = self.get_clock().now().to_msg()
        # out_msg.name = self.joint_names
        # out_msg.velocity = list(q_dot)
        out_msg.layout.dim = []
        out_msg.layout.data_offset = 0
        out_msg.data = list(q_dot)
        
        self.vel_pub.publish(out_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = VelocityNode()
        rclpy.spin(node)
    except InterruptedError:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
