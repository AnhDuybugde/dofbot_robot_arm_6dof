"""Interactive recorder for pre-recorded square routes.

The operator jogs with the robot's existing low-level tool, then captures actual
joint_states here.  This tool never converts square XYZ into joint angles.
"""

from __future__ import annotations

import argparse

import rclpy

from .chess_executor import ChessExecutor, default_routes_path
from .route_database import RouteError
from .trajectory_executor import ExecutionError


HELP = "commands: home | goto q1 q2 q3 q4 q5 | capture | show | pop | clear | save | reload | test | test-reverse | validate | unvalidate | notes <text> | quit"


def main() -> None:
    parser = argparse.ArgumentParser(description="Record one chess square route from actual joint states")
    parser.add_argument("square", help="a1 through h8")
    parser.add_argument("--routes", default=str(default_routes_path()),
                        help="editable route YAML")
    args = parser.parse_args()
    rclpy.init()
    node = ChessExecutor(routes_path=args.routes)
    square = args.square.lower()
    route = [list(node.home["home_joints"])]
    try:
        existing = node.db.get(square)
        if existing.route:
            route = [list(point) for point in existing.route]
        print(f"Recording {square}; database: {node.db.path}")
        print("Route starts at HOME. Jog externally or use goto, then capture each safe waypoint.")
        print(HELP)
        while rclpy.ok():
            try:
                words = input(f"calibrate {square}> ").strip().split()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not words:
                continue
            command = words[0].lower()
            try:
                if command in ("quit", "exit"):
                    break
                if command == "home":
                    node.home_robot()
                elif command == "goto" and len(words) == 6:
                    target = [float(value) for value in words[1:]]
                    node.trajectory_executor.execute_waypoint(
                        target, square=square, index=len(route), label="calibration jog")
                elif command == "capture":
                    # Give /joint_states one short spin if the terminal started instantly.
                    rclpy.spin_once(node, timeout_sec=0.1)
                    actual = node.trajectory_executor.actual_arm()
                    if actual is None:
                        raise ExecutionError("no complete arm joint state to capture")
                    route.append([round(q, 6) for q in actual])
                    print(f"captured waypoint {len(route)-1}: {route[-1]}")
                elif command == "show":
                    for index, point in enumerate(route):
                        print(f"{index}: {point}")
                elif command == "pop":
                    if len(route) > 1:
                        print(f"removed: {route.pop()}")
                elif command == "clear":
                    route = [list(node.home["home_joints"])]
                elif command == "save":
                    node.db.replace_route(square, route)
                    node.db.save()
                    print("saved as UNCALIBRATED; run test, inspect physically, then validate.")
                elif command == "reload":
                    node.db.reload()
                    route = [list(point) for point in node.db.get(square).route] or [list(node.home["home_joints"])]
                elif command in ("test", "test-reverse"):
                    node.db.replace_route(square, route)
                    node.test_square(square, allow_unvalidated=True)
                elif command == "validate":
                    node.db.replace_route(square, route)
                    node.db.mark_validated(square, True)
                    node.db.save()
                    print("VALIDATED: runtime may now execute this route.")
                elif command == "unvalidate":
                    node.db.replace_route(square, route)
                    node.db.save()
                elif command == "notes":
                    node.db.replace_route(square, route, " ".join(words[1:]))
                    print("notes staged; use save or validate to persist.")
                else:
                    print(HELP)
            except (ExecutionError, RouteError, RuntimeError, ValueError) as exc:
                node.stop()
                print(f"ERROR: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
