#!/usr/bin/env python3
"""
Replay robot movements from a rosbag into a live Gazebo simulation.

Strategy:
- SCARA: Only publish when joint positions change significantly (> 0.01 rad).
  This lets the controller complete its trajectory without constant preemption.
- GRIPPER CLOSED: 10Hz timer keeps grip active (only when closed).
- GRIPPER OPEN: Send open command once on close→open transition, then stop.
- CMD_VEL: Replayed directly.

Usage:
    1. Launch Gazebo + controllers (deploy or robot_gazebo + spawn_controllers)
    2. Run: python3 replay_from_bag.py <path_to_rosbag>
"""

import sys
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration

from rosbags.typesys import Stores, get_typestore
from rosbags.rosbag2 import Reader


# Joint-to-controller mapping
SCARA_JOINTS = [
    "lower_link_joint",
    "middle_link_joint",
    "upper_link_joint",
    "end_link_joint",
    "wrist_link_joint",
]

GRIPPER_JOINTS = [
    "left_finger_link_joint",
    "right_finger_link_joint",
]

# Thresholds
GRIPPER_CLOSE_THRESHOLD = 0.04
GRIPPER_FULL_CLOSE = 0.14
SCARA_CHANGE_THRESHOLD = 0.01  # rad — only send if a joint moved this much


class BagReplayer(Node):
    def __init__(self):
        super().__init__("bag_replayer")

        self.scara_pub = self.create_publisher(
            JointTrajectory, "/scara_controller/joint_trajectory", 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Gripper state
        self._gripper_is_closed = False
        self._lock = threading.Lock()

        # Timer ONLY publishes when gripper is closed
        self._gripper_timer = self.create_timer(0.1, self._gripper_hold_callback)

        self.get_logger().info("Bag replayer ready")

    def _gripper_hold_callback(self):
        """10Hz: keep hammering close command ONLY when gripping."""
        with self._lock:
            if not self._gripper_is_closed:
                return

        msg = JointTrajectory()
        msg.joint_names = GRIPPER_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [GRIPPER_FULL_CLOSE, GRIPPER_FULL_CLOSE]
        point.time_from_start = Duration(sec=0, nanosec=200_000_000)
        msg.points = [point]
        self.gripper_pub.publish(msg)

    def set_gripper_closed(self, closed: bool):
        """Update gripper state. Sends open command once on transition."""
        with self._lock:
            was_closed = self._gripper_is_closed
            self._gripper_is_closed = closed

        # Only send open command once on close→open transition
        if was_closed and not closed:
            msg = JointTrajectory()
            msg.joint_names = GRIPPER_JOINTS
            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.0]
            point.time_from_start = Duration(sec=0, nanosec=500_000_000)
            msg.points = [point]
            self.gripper_pub.publish(msg)
            self.get_logger().info("Gripper → OPEN")

        if not was_closed and closed:
            self.get_logger().info("Gripper → CLOSED (timer active)")

    def send_scara_positions(self, positions: dict):
        """Send arm joint positions to the scara controller."""
        msg = JointTrajectory()
        msg.joint_names = SCARA_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [positions.get(j, 0.0) for j in SCARA_JOINTS]
        point.time_from_start = Duration(sec=0, nanosec=500_000_000)
        msg.points = [point]

        self.scara_pub.publish(msg)

    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)


def replay_bag(bag_path: str):
    rclpy.init()
    node = BagReplayer()
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Spin in background for the gripper timer
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.get_logger().info(f"Reading rosbag: {bag_path}")

    messages = []
    with Reader(Path(bag_path)) as reader:
        for conn in reader.connections:
            node.get_logger().info(f"  Found topic: {conn.topic} ({conn.msgtype})")

        for conn, timestamp, rawdata in reader.messages():
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            messages.append((timestamp, conn.topic, msg))

    if not messages:
        node.get_logger().error("No messages found in the rosbag!")
        return

    messages.sort(key=lambda x: x[0])
    start_time = messages[0][0]

    node.get_logger().info(
        f"Loaded {len(messages)} messages, duration: "
        f"{(messages[-1][0] - start_time) / 1e9:.1f}s"
    )
    node.get_logger().info("Starting replay in 3 seconds...")
    time.sleep(3)

    replay_start = time.time()
    # Track last SENT positions (not every reading)
    last_sent_scara = {j: 0.0 for j in SCARA_JOINTS}
    msg_count = 0

    for timestamp, topic, msg in messages:
        elapsed_bag = (timestamp - start_time) / 1e9
        elapsed_real = time.time() - replay_start

        wait_time = elapsed_bag - elapsed_real
        if wait_time > 0:
            time.sleep(wait_time)

        if topic == "/joint_states":
            positions = {}
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    positions[name] = float(msg.position[i])

            # SCARA: only send if any joint changed significantly
            scara_changed = any(
                abs(positions.get(j, 0) - last_sent_scara.get(j, 0))
                > SCARA_CHANGE_THRESHOLD
                for j in SCARA_JOINTS
            )
            if scara_changed:
                new_pos = {
                    j: positions.get(j, last_sent_scara[j]) for j in SCARA_JOINTS
                }
                node.send_scara_positions(new_pos)
                last_sent_scara = new_pos

            # GRIPPER: just detect open/close, timer handles the rest
            any_closing = any(
                positions.get(j, 0.0) > GRIPPER_CLOSE_THRESHOLD for j in GRIPPER_JOINTS
            )
            node.set_gripper_closed(any_closing)

        elif topic == "/cmd_vel":
            node.send_cmd_vel(
                float(msg.linear.x),
                float(msg.angular.z),
            )

        msg_count += 1
        if msg_count % 500 == 0:
            node.get_logger().info(
                f"  Replayed {msg_count}/{len(messages)} messages ({elapsed_bag:.1f}s)"
            )

    node.send_cmd_vel(0.0, 0.0)
    node.get_logger().info(f"Replay complete! {msg_count} messages replayed.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 replay_from_bag.py <path_to_rosbag>")
        sys.exit(1)

    replay_bag(sys.argv[1])
