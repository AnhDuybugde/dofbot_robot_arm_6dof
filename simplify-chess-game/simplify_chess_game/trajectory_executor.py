"""Direct ros2_control action client.  No MoveIt, IK, FK, Cartesian, or OMPL API."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Callable

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint"]


class ExecutionError(RuntimeError):
    pass


class TrajectoryExecutor:
    def __init__(self, node: Node, config: dict, log_step: Callable[..., None]):
        self.node, self.config, self.log_step = node, config, log_step
        self.current: dict[str, float] = {}
        self.state_stamp = 0.0
        self._active_goal = None
        self.node.create_subscription(JointState, config["joint_states_topic"], self._on_state, 10)
        self.client = ActionClient(node, FollowJointTrajectory, config["arm_action"])

    def _on_state(self, message: JointState) -> None:
        self.current.update(dict(zip(message.name, message.position)))
        self.state_stamp = time.monotonic()

    def actual_arm(self) -> list[float] | None:
        if all(name in self.current for name in ARM_JOINTS):
            return [self.current[name] for name in ARM_JOINTS]
        return None

    def wait_ready(self) -> None:
        timeout = float(self.config["action_server_timeout_s"])
        if not self.client.wait_for_server(timeout_sec=timeout):
            raise ExecutionError(f"arm action unavailable: {self.config['arm_action']}")

    def stop(self) -> None:
        """Cancel the active controller action; never synthesize an alternate route."""
        if self._active_goal is not None:
            future = self._active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            self._active_goal = None

    def _duration(self, start: list[float], target: list[float]) -> float:
        delta = max(abs(a - b) for a, b in zip(start, target))
        velocity = float(self.config["max_velocity_rad_s"])
        acceleration = float(self.config["max_acceleration_rad_s2"])
        # Conservative trapezoidal bound; controller also receives velocity/acceleration caps.
        return max(float(self.config["min_waypoint_duration_s"]), delta / velocity,
                   2.0 * math.sqrt(delta / acceleration) if delta else 0.0)

    @staticmethod
    def _duration_msg(seconds: float) -> Duration:
        msg = Duration()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - msg.sec) * 1_000_000_000)
        return msg

    def execute_waypoint(self, target: list[float], *, square: str, index: int,
                         label: str) -> None:
        if len(target) != 5:
            raise ExecutionError("target must contain arm1..arm5")
        self.wait_ready()
        start = self.actual_arm()
        if start is None:
            raise ExecutionError("no complete /joint_states received; refusing blind execution")
        duration = self._duration(start, target)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(q) for q in target]
        # These fields are setpoints, not limits: use signed velocities so a
        # negative joint move is not accidentally sent a positive velocity.
        point.velocities = [
            max(-float(self.config["max_velocity_rad_s"]),
                min(float(self.config["max_velocity_rad_s"]),
                    (target_q - start_q) / duration))
            for start_q, target_q in zip(start, target)
        ]
        # Leave acceleration unconstrained at the point level; the controller's
        # configured limits remain authoritative and avoid a false sign claim.
        point.time_from_start = self._duration_msg(duration)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = self._duration_msg(1.0)
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future,
                                         timeout_sec=float(self.config["waypoint_timeout_s"]))
        handle = future.result()
        if handle is None or not handle.accepted:
            self.log_step(square=square, trajectory_index=index, target=target,
                          actual=self.actual_arm(), label=label, success=False)
            raise ExecutionError(f"{label}: controller rejected waypoint {index}")
        self._active_goal = handle
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future,
                                         timeout_sec=float(self.config["waypoint_timeout_s"]))
        result = result_future.result()
        if result is None:
            # The only safe continuation after a timeout is no continuation.
            self.stop()
        self._active_goal = None
        actual = self.actual_arm()
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        if actual is not None:
            ok = ok and all(abs(a - b) <= float(self.config["endpoint_tolerance_rad"])
                            for a, b in zip(actual, target))
        self.log_step(square=square, trajectory_index=index, target=target,
                      actual=actual, label=label, success=ok)
        if not ok:
            raise ExecutionError(f"{label}: waypoint {index} failed; execution stopped")

    def execute_route(self, route: list[list[float]], *, square: str, label: str) -> None:
        # Point 0 is HOME, so it is commanded too: this confirms the stated invariant.
        for index, target in enumerate(route):
            self.execute_waypoint(target, square=square, index=index, label=label)
