import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from roboticstoolbox import DHRobot, RevoluteDH
from sensor_msgs.msg import JointState


class OvisVizNode(Node):
    def __init__(self):
        super().__init__('ovis_viz_node')

        self.robot = DHRobot(
            [
                RevoluteDH(d=0.10001, a=0.0, alpha=np.pi / 2),
                RevoluteDH(d=0.0, a=-0.30039, alpha=np.pi),
                RevoluteDH(d=0.0, a=0.0, alpha=np.pi / 2, offset=np.pi / 2),
                RevoluteDH(d=0.33480, a=0.0, alpha=-np.pi / 2),
                RevoluteDH(d=0.0, a=0.0, alpha=np.pi / 2),
                RevoluteDH(d=0.07000, a=0.0, alpha=0.0),
            ],
            name='ovis',
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/robot_joint_states',
            self.joint_update,
            10,
        )

        self.joint_names = [
            'ovis_joint_1',
            'ovis_joint_2',
            'ovis_joint_3',
            'ovis_joint_4',
            'ovis_joint_5',
            'ovis_joint_6',
        ]
        self.q = np.zeros(6)
        self.timer = self.create_timer(0.1, self.plot_robot)

    def joint_update(self, msg: JointState):
        if not msg.position:
            return

        index = {name: i for i, name in enumerate(msg.name)}
        try:
            self.q = np.array([msg.position[index[name]] for name in self.joint_names])
        except KeyError:
            if len(msg.position) >= 6:
                self.q = np.array(msg.position[:6])

    def plot_robot(self):
        self.robot.plot(self.q, block=False)
        ax = plt.gca()
        ax.set_xlim([-1.0, 1.0])
        ax.set_ylim([-1.0, 1.0])
        ax.set_zlim([-1.0, 1.0])
        plt.draw()


def main(args=None):
    rclpy.init(args=args)
    node = OvisVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
