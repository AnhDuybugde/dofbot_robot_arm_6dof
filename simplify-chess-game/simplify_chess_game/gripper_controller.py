from __future__ import annotations

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node

from .trajectory_executor import ExecutionError


class GripperController:
    def __init__(self, node: Node, config: dict, gripper: dict, log_step):
        self.node, self.config, self.gripper, self.log_step = node, config, gripper, log_step
        self.client = ActionClient(node, GripperCommand, config["gripper_action"])

    def set(self, state: str, square: str = "-") -> None:
        if state not in ("PRE_CLOSE", "CLOSE"):
            raise ExecutionError(f"unsupported gripper state {state}")
        if not self.client.wait_for_server(timeout_sec=float(self.config["action_server_timeout_s"])):
            raise ExecutionError(f"gripper action unavailable: {self.config['gripper_action']}")
        value = float(self.gripper[f"GRIPPER_{state}"])
        goal = GripperCommand.Goal()
        goal.command.position = value
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future,
                                         timeout_sec=float(self.config["gripper_timeout_s"]))
        handle = future.result()
        if handle is None or not handle.accepted:
            self.log_step(square=square, trajectory_index=None, target=None, actual=None,
                          gripper_state=state, label="gripper", success=False)
            raise ExecutionError(f"gripper {state} rejected")
        outcome = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, outcome,
                                         timeout_sec=float(self.config["gripper_timeout_s"]))
        result = outcome.result()
        if result is None:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=1.0)
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        self.log_step(square=square, trajectory_index=None, target=None, actual=None,
                      gripper_state=state, label="gripper", success=ok)
        if not ok:
            raise ExecutionError(f"gripper {state} failed; execution stopped")
