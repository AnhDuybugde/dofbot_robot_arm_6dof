"""Generate replay routes once with MoveIt; runtime never imports this module.

This is deliberately an offline tool.  It plans HOME -> square with MoveGroup,
extracts the resulting arm trajectory, and stores it in square_routes.yaml.
The default output remains UNCALIBRATED until the route is checked on hardware.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, RobotState
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import yaml

from .chess_executor import config_path, default_routes_path, load_yaml
from .route_database import SQUARES

ARM_JOINTS = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint"]
BOARD_ORIGIN = (0.1105, -0.0915)
SQUARE_SIZE = 0.026
DEFAULT_TCP_Z = 0.058


def compress(route: list[list[float]], maximum: int) -> list[list[float]]:
    if maximum < 2 or len(route) <= maximum:
        return route
    indices = [round(i * (len(route) - 1) / (maximum - 1)) for i in range(maximum)]
    return [route[index] for index in indices]


class OfflinePlanner(Node):
    def __init__(self, allowed_time: float):
        super().__init__("simplify_offline_ik_calibrator")
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.allowed_time = allowed_time

    def plan_square(self, square: str, z: float) -> list[list[float]] | None:
        file_index = "abcdefgh".index(square[0])
        rank_index = int(square[1]) - 1
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = BOARD_ORIGIN[0] + rank_index * SQUARE_SIZE
        pose.pose.position.y = BOARD_ORIGIN[1] + file_index * SQUARE_SIZE
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.004, 0.004, 0.004]
        position = PositionConstraint()
        position.header = pose.header
        position.link_name = "Gripping_point_Link"
        position.constraint_region.primitives = [primitive]
        position.constraint_region.primitive_poses = [pose.pose]
        position.weight = 1.0

        request = MoveGroup.Goal().request
        request.group_name = "arm_group"
        request.num_planning_attempts = 3
        request.allowed_planning_time = self.allowed_time
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.start_state = RobotState()
        request.start_state.joint_state = JointState(name=ARM_JOINTS, position=[0.0] * 5)
        request.goal_constraints = [Constraints(position_constraints=[position])]

        goal = MoveGroup.Goal()
        goal.request = request
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=self.allowed_time + 30.0)
        wrapped = result_future.result()
        if wrapped is None or wrapped.result.error_code.val != 1:
            return None
        trajectory = wrapped.result.planned_trajectory.joint_trajectory
        by_name = {name: index for index, name in enumerate(trajectory.joint_names)}
        if any(name not in by_name for name in ARM_JOINTS):
            return None
        route = [[0.0] * 5]
        for point in trajectory.points:
            values = [float(point.positions[by_name[name]]) for name in ARM_JOINTS]
            if all(math.isfinite(value) for value in values):
                if values != route[-1]:
                    route.append(values)
        return route if len(route) >= 2 else None


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline MoveIt route recorder")
    parser.add_argument("--routes", default=str(default_routes_path()))
    parser.add_argument("--square", help="only one square, e.g. b2")
    parser.add_argument("--z", type=float, default=DEFAULT_TCP_Z)
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--validate", action="store_true",
                        help="mark successful MoveIt routes VALIDATED (simulation only)")
    parser.add_argument("--validate-existing", action="store_true",
                        help="promote already populated routes without replanning")
    parser.add_argument("--compress-existing", type=int, metavar="N",
                        help="compress populated routes to at most N waypoints")
    args = parser.parse_args()
    squares = [args.square.lower()] if args.square else list(SQUARES)
    data = load_yaml(args.routes)
    routes = data.setdefault("routes", {})
    if args.compress_existing is not None:
        if args.compress_existing < 2:
            parser.error("--compress-existing must be at least 2")
        changed = 0
        for square in SQUARES:
            entry = routes.get(square, {})
            if entry.get("route") and len(entry["route"]) > args.compress_existing:
                entry["route"] = compress(entry["route"], args.compress_existing)
                entry["notes"] = (entry.get("notes", "") +
                                   f"; compressed offline to <= {args.compress_existing} waypoints").strip("; ")
                changed += 1
        Path(args.routes).write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        print(f"compressed {changed}/64 routes")
        return 0
    if args.validate_existing:
        promoted = 0
        for square in SQUARES:
            entry = routes.get(square, {})
            if entry.get("route"):
                entry["status"] = "VALIDATED"
                entry["validated"] = True
                entry["notes"] = ((entry.get("notes", "") +
                                   "; offline MoveIt trajectory; verify on hardware")
                                  .strip("; "))
                promoted += 1
        Path(args.routes).write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        print(f"promoted {promoted}/64 populated routes to VALIDATED (offline only)")
        return 0 if promoted == 64 else 1
    rclpy.init()
    node = OfflinePlanner(args.planning_time)
    try:
        if not node.client.wait_for_server(timeout_sec=15.0):
            print("ERROR: /move_action unavailable")
            return 2
        for square in squares:
            if square not in SQUARES:
                print(f"ERROR: invalid square {square}")
                continue
            route = node.plan_square(square, args.z)
            if route is None:
                print(f"FAIL {square}: no MoveIt trajectory")
                continue
            routes[square] = {
                "status": "VALIDATED" if args.validate else "UNCALIBRATED",
                "validated": bool(args.validate), "route": route,
                "notes": "offline MoveIt generated; verify collision and hardware before use",
            }
            print(f"OK {square}: {len(route)} waypoints")
        Path(args.routes).write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
