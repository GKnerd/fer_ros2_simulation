#!/usr/bin/env python3
"""
Send WAYPOINT_SET_4 to joint_effort_traj_controller as JointTrajectory goals.
Each segment duration matches the MPC default (7 s).

Usage:
    python3 ~/ros2_ws/pid_waypoint_runner.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

JOINT_NAMES = [
    "fer_joint1", "fer_joint2", "fer_joint3",
    "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7",
]

TRAJ_DURATION = 7.0  # seconds per segment — matches MPC trajectory_duration default

# WAYPOINT_SET_4 — same as mpc_coupled_qp_node
WAYPOINTS = [
    ( 0.8,  -0.5,    0.5, -2.8,    0.5,  1.2,    0.3   ),  # A
    ( 0.0,  -1.0,    0.0, -1.5,    0.0,  2.5,    1.5   ),  # B
    (-0.8,  -0.5,   -0.5, -2.8,   -0.5,  1.2,    0.3   ),  # C
    ( 0.0,  -1.3,    0.8, -2.0,   -0.5,  1.8,    0.6   ),  # D
    ( 0.4,  -0.7,   -0.8, -2.2,    0.8,  1.6,    1.0   ),  # E
    ( 0.0,  -0.7854, 0.0, -2.3562, 0.0,  1.5708, 0.7854),  # home
]
LABELS = ["A", "B", "C", "D", "E", "home"]


class PIDRunner(Node):
    def __init__(self):
        super().__init__("pid_waypoint_runner")
        self._client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_effort_traj_controller/follow_joint_trajectory"
        )
        self.get_logger().info("Waiting for /joint_effort_traj_controller/follow_joint_trajectory ...")
        self._client.wait_for_server()
        self.get_logger().info("Controller ready.")

    def send_waypoint(self, label, positions):
        from trajectory_msgs.msg import JointTrajectory
        from builtin_interfaces.msg import Duration

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.velocities = [0.0] * 7
        secs = int(TRAJ_DURATION)
        nsecs = int((TRAJ_DURATION - secs) * 1e9)
        pt.time_from_start.sec = secs
        pt.time_from_start.nanosec = nsecs
        goal.trajectory.points = [pt]

        self.get_logger().info(f"  Sending waypoint {label} ...")
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error(f"  {label}: goal REJECTED")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        ok = (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        self.get_logger().info(f"  {label}: {'OK' if ok else 'FAILED (code=' + str(result.error_code) + ')'}")
        return ok

    def run(self):
        for label, positions in zip(LABELS, WAYPOINTS):
            if not rclpy.ok():
                break
            self.send_waypoint(label, positions)
        self.get_logger().info("All waypoints sent.")


def main():
    rclpy.init()
    node = PIDRunner()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
