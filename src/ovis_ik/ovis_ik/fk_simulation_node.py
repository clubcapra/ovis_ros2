import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateBridgeNode(Node):
    def __init__(self):
        super().__init__('fk_simulation_node')
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10,
        )
        self.state_pub = self.create_publisher(JointState, '/robot_joint_states', 10)

        self.joint_names = [
            'ovis_joint_1',
            'ovis_joint_2',
            'ovis_joint_3',
            'ovis_joint_4',
            'ovis_joint_5',
            'ovis_joint_6',
        ]

    def joint_callback(self, msg: JointState):
        if not msg.position:
            return

        index = {name: i for i, name in enumerate(msg.name)}
        try:
            ordered_pos = [msg.position[index[name]] for name in self.joint_names]
            ordered_vel = [msg.velocity[index[name]] for name in self.joint_names] if msg.velocity else []
        except KeyError:
            if len(msg.position) < 6:
                return
            ordered_pos = list(msg.position[:6])
            ordered_vel = list(msg.velocity[:6]) if msg.velocity else []

        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.name = self.joint_names
        out_msg.position = ordered_pos
        out_msg.velocity = ordered_vel
        self.state_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
