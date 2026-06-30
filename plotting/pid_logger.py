#!/usr/bin/env python3
"""
Log joint_effort_traj_controller state to a CSV matching the MPC log format.
Detects waypoint transitions by watching when the desired joint positions change.

Columns written: time, q1-7, dq1-7, qref1-7, error1-7, current_waypoint

Usage:
    python3 ~/ros2_ws/pid_logger.py
    (run while pid_waypoint_runner.py is sending goals)
    Ctrl-C to stop — CSV is flushed on exit.

Output: ~/ros2_ws/pid_log.csv
"""

import sys
import csv
import signal
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

OUTPUT_PATH = "/home/fer_ros2_sim/ros2_ws/pid_log.csv"
TOPIC       = "/joint_effort_traj_controller/controller_state"
N_JOINTS    = 7
# Threshold to detect a new waypoint goal (max position change across joints)
WAYPOINT_JUMP_THRESH = 0.05  # rad


class PIDLogger(Node):
    def __init__(self):
        super().__init__("pid_logger")

        self._file   = open(OUTPUT_PATH, "w", newline="")
        self._writer = csv.writer(self._file)

        header = (
            ["time"]
            + [f"q{j+1}"    for j in range(N_JOINTS)]
            + [f"dq{j+1}"   for j in range(N_JOINTS)]
            + [f"qref{j+1}" for j in range(N_JOINTS)]
            + [f"error{j+1}" for j in range(N_JOINTS)]
            + ["current_waypoint"]
        )
        self._writer.writerow(header)

        self._current_waypoint = 0
        self._last_desired     = None
        self._start_time       = None

        self._sub = self.create_subscription(
            JointTrajectoryControllerState,
            TOPIC,
            self._cb,
            10,
        )
        self.get_logger().info(f"Logging {TOPIC} → {OUTPUT_PATH}")

    def _cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._start_time is None:
            self._start_time = now
        t = now - self._start_time

        desired = list(msg.reference.positions)
        actual  = list(msg.feedback.positions)
        vel     = list(msg.feedback.velocities)
        error   = list(msg.error.positions)

        if len(desired) < N_JOINTS:
            return  # controller not yet active

        # Detect waypoint switch: desired positions jumped
        if self._last_desired is not None:
            jump = max(abs(desired[j] - self._last_desired[j]) for j in range(N_JOINTS))
            if jump > WAYPOINT_JUMP_THRESH:
                self._current_waypoint += 1
                self.get_logger().info(
                    f"Waypoint → {self._current_waypoint}  (jump={jump:.3f} rad)"
                )
        self._last_desired = desired

        row = (
            [round(t, 5)]
            + [round(v, 8) for v in actual[:N_JOINTS]]
            + [round(v, 8) for v in vel[:N_JOINTS]]
            + [round(v, 8) for v in desired[:N_JOINTS]]
            + [round(v, 8) for v in error[:N_JOINTS]]
            + [self._current_waypoint]
        )
        self._writer.writerow(row)

    def shutdown(self):
        self._file.flush()
        self._file.close()
        self.get_logger().info(f"CSV saved: {OUTPUT_PATH}")


def main():
    rclpy.init()
    node = PIDLogger()

    def _handler(sig, frame):
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
