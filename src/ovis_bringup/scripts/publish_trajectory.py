#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class PublishTrajectoryNode(Node):
    def __init__(self):
        super().__init__('publish_trajectory_node')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.target_positions = [-1.0, 0.0, 0.0, -1.0, 0.5, 0.0]
        self.target_reached = False

        # Timer to publish the trajectory
        self.timer = self.create_timer(1.0, self.publish_trajectory)

    def publish_trajectory(self):
        if not self.target_reached:
            msg = JointTrajectory()
            msg.joint_names = ['ovis_joint_2', 'ovis_joint_4', 'ovis_joint_1', 'ovis_joint_3', 'ovis_joint_5', 'ovis_joint_6']
            point = JointTrajectoryPoint()
            point.positions = self.target_positions
            point.time_from_start.sec = 5
            point.time_from_start.nanosec = 0
            msg.points.append(point)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing trajectory: "%s"' % msg)
        else:
            self.get_logger().info('Target position reached. Stopping publishing and shutting down.')
            raise SystemExit

    def joint_states_callback(self, msg):
        current_positions = msg.position
        tolerance = 0.01
        if all(abs(cp - tp) < tolerance for cp, tp in zip(current_positions, self.target_positions)):
            self.target_reached = True

def main(args=None):
    rclpy.init(args=args)
    node = PublishTrajectoryNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Node is shutting down')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
