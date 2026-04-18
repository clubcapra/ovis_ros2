#!/usr/bin/env python3
"""
Ovis Arm - Cartesian Velocity IK Node (ROS2 Humble)
=====================================================
Subscribes to a TwistStamped (Cartesian end-effector velocity) and
publishes Float64MultiArray joint velocities using Damped Least Squares IK.

Topics
------
Subscribed:
  /robot_description         (std_msgs/String)    — one-shot, to build the model
  /joint_states              (sensor_msgs/JointState) — current joint positions
  /ovis/cmd_cartesian_vel    (geometry_msgs/TwistStamped) — desired EE velocity

Published:
  /ovis_controller/commands  (std_msgs/Float64MultiArray) — [j1..j6] rad/s,
                               order matches ForwardCommandController joints list

Parameters
----------
  prefix          (str,   default "ovis")    — xacro prefix used in joint names
  end_effector    (str,   default "ovis_end_effector") — pinocchio frame name
  base_frame      (str,   default "world")   — root frame for Jacobian
  damping         (float, default 0.05)      — DLS damping factor λ
  max_joint_vel   (float, default 1.0)       — rad/s clamp on output
  cmd_timeout     (float, default 0.5)       — s, zero output if no cmd received
  publish_rate    (float, default 100.0)     — Hz of the control loop

Dependencies
------------
  pip install pin numpy          (pinocchio ROS2 package: ros-humble-pinocchio)
  ros-humble-{rclpy, std-msgs, sensor-msgs, geometry-msgs}
"""

import numpy as np
import pinocchio as pin
from io import StringIO
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped


class OvisVelocityIK(Node):

    def __init__(self):
        super().__init__("ovis_velocity_ik")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("prefix",        "ovis")
        self.declare_parameter("end_effector",  "ovis_end_effector")
        self.declare_parameter("base_frame",    "world")
        self.declare_parameter("damping",       0.05)
        self.declare_parameter("max_joint_vel", 1.0)
        self.declare_parameter("cmd_timeout",   0.5)
        self.declare_parameter("publish_rate",  100.0)
        # Must match the joints: list in the ForwardCommandController config,
        # in the same order — this defines the output array ordering.
        self.declare_parameter("controller_joints", [
            "ovis_joint_1", "ovis_joint_2", "ovis_joint_3",
            "ovis_joint_4", "ovis_joint_5", "ovis_joint_6",
        ])

        self.prefix       = self.get_parameter("prefix").value
        self.ee_frame_name = self.get_parameter("end_effector").value
        self.base_frame   = self.get_parameter("base_frame").value
        self.damping      = self.get_parameter("damping").value
        self.max_vel      = self.get_parameter("max_joint_vel").value
        self.cmd_timeout  = self.get_parameter("cmd_timeout").value
        rate_hz           = self.get_parameter("publish_rate").value
        # Output order must match the ForwardCommandController joints: list exactly
        self.controller_joints = self.get_parameter("controller_joints").value

        # ── Internal state ────────────────────────────────────────────────────
        self._lock          = threading.Lock()
        self._model         = None          # pinocchio model (set once)
        self._data          = None          # pinocchio data
        self._ee_frame_id   = None
        self._joint_names   = []            # ordered actuated joint names
        self._q             = None          # current joint positions (np array)
        self._cart_vel      = np.zeros(6)   # [vx vy vz wx wy wz]
        self._last_cmd_time = 0.0

        # ── robot_description QoS: latched (transient_local) ─────────────────
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self._desc_sub = self.create_subscription(
            String, "/robot_description",
            self._robot_description_cb, latched_qos)

        self._js_sub = self.create_subscription(
            JointState, "/joint_states",
            self._joint_states_cb, 10)

        self._cmd_sub = self.create_subscription(
            TwistStamped, "/ovis/cmd_cartesian_vel",
            self._cmd_vel_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._vel_pub = self.create_publisher(
            Float64MultiArray, "/ovis_controller/commands", 10)

        # ── Control loop timer ────────────────────────────────────────────────
        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"Ovis velocity IK node started. "
            f"Waiting for /robot_description …  (prefix='{self.prefix}')"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _robot_description_cb(self, msg: String):
        """Build the pinocchio model from the URDF string (called once)."""
        with self._lock:
            if self._model is not None:
                return  # already loaded

        self.get_logger().info("robot_description received, building pinocchio model …")
        try:
            model = pin.buildModelFromXML(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
            return

        data = model.createData()

        # Locate the end-effector frame
        if not model.existFrame(self.ee_frame_name):
            # Try with prefix (sometimes the EE is a body, not a frame)
            self.get_logger().warn(
                f"Frame '{self.ee_frame_name}' not found. "
                f"Available frames: {[model.frames[i].name for i in range(model.nframes)]}"
            )
            return

        ee_frame_id = model.getFrameId(self.ee_frame_name)

        # Collect the 6 actuated joints in model order (skip universe/fixed)
        joint_names = []
        for jid in range(1, model.njoints):          # 0 = universe
            name = model.names[jid]
            # Only keep joints that belong to our arm (prefix filter)
            if name.startswith(f"{self.prefix}_joint_"):
                joint_names.append(name)

        if len(joint_names) != 6:
            self.get_logger().error(
                f"Expected 6 actuated joints with prefix '{self.prefix}_joint_', "
                f"found {len(joint_names)}: {joint_names}"
            )
            return

        self.get_logger().info(
            f"Pinocchio model ready. "
            f"nq={model.nq}, nv={model.nv}, "
            f"EE frame='{self.ee_frame_name}' (id={ee_frame_id})\n"
            f"Joint order: {joint_names}"
        )

        with self._lock:
            self._model       = model
            self._data        = data
            self._ee_frame_id = ee_frame_id
            self._joint_names = joint_names
            self._q           = pin.neutral(model)

    def _joint_states_cb(self, msg: JointState):
        """Update current joint positions from /joint_states."""
        with self._lock:
            if self._model is None or self._q is None:
                return
            # Map by name — /joint_states order may differ from model order
            for idx, name in enumerate(msg.name):
                if name in self._joint_names:
                    jid = self._model.getJointId(name)
                    qidx = self._model.joints[jid].idx_q
                    self._q[qidx] = msg.position[idx]

    def _cmd_vel_cb(self, msg: TwistStamped):
        """Cache the latest Cartesian velocity command."""
        with self._lock:
            self._cart_vel = np.array([
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
                msg.twist.angular.x,
                msg.twist.angular.y,
                msg.twist.angular.z,
            ])
            self._last_cmd_time = time.monotonic()

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        with self._lock:
            if self._model is None or self._q is None:
                return  # model not ready yet

            # Zero output if command has timed out
            age = time.monotonic() - self._last_cmd_time
            if age > self.cmd_timeout:
                self._publish_zeros()
                return

            q        = self._q.copy()
            cart_vel = self._cart_vel.copy()
            model    = self._model
            data     = self._data
            frame_id = self._ee_frame_id
            lam      = self.damping

        # ── Forward kinematics + Jacobian ─────────────────────────────────────
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        # 6×nv Jacobian in LOCAL frame — velocity commands are interpreted in the
        # end-effector frame, so "forward" always means along the tool axis
        # regardless of the arm's current configuration.
        # Switch to LOCAL_WORLD_ALIGNED if you ever want world-frame commands.
        J = pin.computeFrameJacobian(
            model, data, q, frame_id,
            pin.ReferenceFrame.LOCAL
        )

        # ── Damped Least Squares inverse ──────────────────────────────────────
        # q̇ = Jᵀ (J Jᵀ + λ²I)⁻¹ ẋ
        lam2 = lam ** 2
        A    = J @ J.T + lam2 * np.eye(6)
        qdot_full = J.T @ np.linalg.solve(A, cart_vel)

        # ── Extract velocities in ForwardCommandController order ──────────────
        # controller_joints defines the exact array order the controller expects.
        joint_vels = np.zeros(len(self.controller_joints))
        for out_idx, name in enumerate(self.controller_joints):
            jid  = model.getJointId(name)
            vidx = model.joints[jid].idx_v
            joint_vels[out_idx] = qdot_full[vidx]

        # ── Clamp ─────────────────────────────────────────────────────────────
        joint_vels = np.clip(joint_vels, -self.max_vel, self.max_vel)

        # ── Publish ───────────────────────────────────────────────────────────
        out = Float64MultiArray()
        out.data = joint_vels.tolist()
        self._vel_pub.publish(out)

    def _publish_zeros(self):
        out = Float64MultiArray()
        out.data = [0.0] * len(self.controller_joints)
        self._vel_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OvisVelocityIK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()