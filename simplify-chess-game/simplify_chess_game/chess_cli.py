from __future__ import annotations

import argparse

import rclpy

from .chess_executor import ChessExecutor
from .trajectory_executor import ExecutionError


HELP = "commands: home | test <square> | test-reverse <square> | status | list-routes | reset-pieces | <source> <target> | quit"


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrated chess route replay (no runtime planning/IK)")
    parser.add_argument("--routes", help="editable square_routes.yaml (default: installed config)")
    parser.add_argument("--speed", type=float, default=2.0,
                        help="motion speed multiplier 0.2..5.0 (default: 2.0)")
    args = parser.parse_args()
    rclpy.init()
    node = ChessExecutor(routes_path=args.routes, speed_multiplier=args.speed)
    print(HELP)
    try:
        while rclpy.ok():
            try:
                command = input("move> ").strip().lower().split()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not command:
                continue
            try:
                if command[0] in ("quit", "exit"):
                    break
                if command == ["home"]:
                    node.home_robot()
                elif command == ["status"]:
                    print(node.status())
                elif command == ["reset-pieces"]:
                    node.reset_pieces()
                    print("piece display reset to initial setup")
                elif command == ["list-routes"]:
                    print("\n".join(node.list_routes()))
                elif len(command) == 2 and command[0] == "test":
                    node.test_square(command[1])
                elif len(command) == 2 and command[0] == "test-reverse":
                    node.test_square(command[1], reverse_only=True)
                elif len(command) == 2:
                    node.move(command[0], command[1])
                else:
                    print(HELP)
            except (ExecutionError, RuntimeError) as exc:
                node.stop()
                print(f"ERROR: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
