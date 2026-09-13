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

    def wait_for_joint_state(self) -> list[float]:
        """Allow the subscription callback to receive a complete state."""
        deadline = time.monotonic() + float(self.config["action_server_timeout_s"])
        while time.monotonic() < deadline:
            actual = self.actual_arm()
            if actual is not None:
                return actual
            rclpy.spin_once(self.node, timeout_sec=0.1)
        raise ExecutionError("no complete /joint_states received; refusing blind execution")

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

    def speed(self) -> float:
        try:
            value = float(self.config.get("speed_multiplier", 1.0))
        except (TypeError, ValueError):
            value = 1.0
        return max(0.2, min(5.0, value))

    def _duration(self, start: list[float], target: list[float]) -> float:
        delta = max(abs(a - b) for a, b in zip(start, target))
        velocity = float(self.config["max_velocity_rad_s"])
        acceleration = float(self.config["max_acceleration_rad_s2"])
        # Conservative trapezoidal bound; controller also receives velocity/acceleration caps.
        raw = max(float(self.config["min_waypoint_duration_s"]), delta / velocity,
                  2.0 * math.sqrt(delta / acceleration) if delta else 0.0)
        return raw / self.speed()

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
        start = self.wait_for_joint_state()
        duration = self._duration(start, target)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(q) for q in target]
        # Stop-and-go từng waypoint: endpoint velocity = 0 để
        # joint_trajectory_controller (splines) không abort goal đoạn ngắn
        # vì phải tới đích với vận tốc khác 0 (như case b2 index 28:
        # lệch chỉ ~0.01 rad nhưng result ABORTED, tay đứng yên).
        # Tốc độ vẫn được giới hạn bởi duration tính từ max_velocity.
        point.velocities = [0.0] * len(target)
        # Leave acceleration unconstrained at the point level; the controller's
        # configured limits remain authoritative and avoid a false sign claim.
        point.time_from_start = self._duration_msg(duration)
        goal.trajectory.points = [point]
        # Nới goal_time_tolerance 1.0 -> 3.0s: VM 100Hz jitter vẫn fail-loud
        # khi lỗi thật, chỉ không abort oan goal đoạn ngắn tới trễ chút.
        goal.goal_time_tolerance = self._duration_msg(3.0)
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
        max_err = (max(abs(a - b) for a, b in zip(actual, target))
                   if actual is not None else float("nan"))
        if actual is not None:
            ok = ok and max_err <= float(self.config["endpoint_tolerance_rad"])
        self.log_step(square=square, trajectory_index=index, target=target,
                      actual=actual, label=label, success=ok)
        if not ok:
            if result is None:
                reason = "timeout waiting for controller result"
            else:
                reason = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                          GoalStatus.STATUS_ABORTED: "ABORTED",
                          GoalStatus.STATUS_CANCELED: "CANCELED"}.get(
                              result.status, f"status={result.status}")
            raise ExecutionError(
                f"{label}: waypoint {index} failed ({reason}, "
                f"max_joint_err={max_err:.4f} rad); execution stopped")

    @staticmethod
    def decimate(route: list[list[float]], threshold: float) -> list[list[float]]:
        """Pure helper: drop near-duplicate intermediate waypoints.

        Keeps the first and last points; drops a middle point when every
        joint is within `threshold` rad of the last kept point. Calibration
        records dense points, so this cuts segment count (each costs at
        least min_waypoint_duration) while preserving path shape.
        """
        cleaned = [[float(q) for q in point] for point in route]
        if len(cleaned) <= 2 or threshold <= 0.0:
            return cleaned
        kept = [cleaned[0]]
        for point in cleaned[1:-1]:
            if max(abs(a - b) for a, b in zip(point, kept[-1])) >= threshold:
                kept.append(point)
        kept.append(cleaned[-1])
        return kept

    @staticmethod
    def build_route_points(route: list[list[float]], start: list[float],
                           duration_fn) -> tuple[list[tuple[list[float], float]], float]:
        """Pure helper: cumulative time_from_start per waypoint. Unit-testable."""
        timed: list[tuple[list[float], float]] = []
        total = 0.0
        previous = [float(q) for q in start]
        for target in route:
            point = [float(q) for q in target]
            total += duration_fn(previous, point)
            timed.append((point, total))
            previous = point
        return timed, total

    def execute_route(self, route: list[list[float]], *, square: str, label: str) -> None:
        """Send one leg as a SINGLE multi-point goal instead of one goal per waypoint.

        Previously every waypoint was its own action goal (>=0.7s + handshake each),
        so a 40-point leg cost ~40 round-trips. Batching keeps the exact same
        waypoints/durations but one handshake per leg: typically 4-6x faster.
        Intermediate points leave velocity unconstrained for smooth blending;
        only the final point stops (velocity 0), which also avoids the
        short-segment ABORT seen with nonzero endpoint velocities (b2 index 28).
        Near-duplicate waypoints are decimated first (config
        decimate_threshold_rad); the endpoint is always kept and verified.
        """
        if not route:
            raise ExecutionError(f"{label}: empty route for {square}")
        for target in route:
            if len(target) != 5:
                raise ExecutionError("target must contain arm1..arm5")
        self.wait_ready()
        start = self.wait_for_joint_state()
        # Drop leading points we already stand on (routes start at HOME and
        # chained legs revisit it); the endpoint is never dropped.
        while len(route) > 1 and max(abs(a - b) for a, b in zip(route[0], start)) < 1e-3:
            route = route[1:]
        original = len(route)
        route = self.decimate(route, float(self.config.get("decimate_threshold_rad", 0.0)))
        if len(route) == 1:
            self.execute_waypoint(route[0], square=square, index=0, label=label)
            return
        print(f"{label}: {original} -> {len(route)} waypoints after decimation")
        timed, total = self.build_route_points(route, start, self._duration)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        for point_positions, time_from_start in timed[:-1]:
            point = JointTrajectoryPoint()
            point.positions = point_positions
            point.time_from_start = self._duration_msg(time_from_start)
            goal.trajectory.points.append(point)
        final_positions, final_time = timed[-1]
        final = JointTrajectoryPoint()
        final.positions = final_positions
        final.velocities = [0.0] * len(final_positions)
        final.time_from_start = self._duration_msg(final_time)
        goal.trajectory.points.append(final)
        # Nới goal_time_tolerance 1.0 -> 3.0s: VM 100Hz jitter vẫn fail-loud
        # khi lỗi thật, chỉ không abort oan goal đoạn ngắn tới trễ chút.
        goal.goal_time_tolerance = self._duration_msg(3.0)
        timeout = total + float(self.config.get(
            "route_timeout_s", self.config["waypoint_timeout_s"]))
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future,
                                         timeout_sec=float(self.config["waypoint_timeout_s"]))
        handle = future.result()
        if handle is None or not handle.accepted:
            self.log_step(square=square, trajectory_index=len(route) - 1,
                          target=route[-1], actual=self.actual_arm(),
                          label=label, success=False)
            raise ExecutionError(f"{label}: controller rejected route ({len(route)} points)")
        self._active_goal = handle
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)
        result = result_future.result()
        if result is None:
            self.stop()
        self._active_goal = None
        actual = self.actual_arm()
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        max_err = (max(abs(a - b) for a, b in zip(actual, route[-1]))
                   if actual is not None else float("nan"))
        if actual is not None:
            ok = ok and max_err <= float(self.config["endpoint_tolerance_rad"])
        self.log_step(square=square, trajectory_index=len(route) - 1, target=route[-1],
                      actual=actual, label=label, success=ok)
        if not ok:
            if result is None:
                reason = "timeout waiting for controller result"
            else:
                reason = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                          GoalStatus.STATUS_ABORTED: "ABORTED",
                          GoalStatus.STATUS_CANCELED: "CANCELED"}.get(
                              result.status, f"status={result.status}")
            raise ExecutionError(
                f"{label}: route failed ({reason}, "
                f"max_joint_err={max_err:.4f} rad); execution stopped")
