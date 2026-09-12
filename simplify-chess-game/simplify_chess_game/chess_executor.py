"""Deterministic HOME -> PICK -> HOME -> DROP -> HOME state machine."""

from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import yaml

from .gripper_controller import GripperController
from .route_database import RouteDatabase
from .trajectory_executor import ExecutionError, TrajectoryExecutor


def config_path(name: str) -> Path:
    return Path(get_package_share_directory("simplify_chess_game")) / "config" / name


def default_routes_path() -> Path:
    """Prefer the checkout config so the calibration CLI can save without sudo."""
    checkout = Path.cwd() / "simplify-chess-game" / "config" / "square_routes.yaml"
    return checkout if checkout.is_file() else config_path("square_routes.yaml")


def load_yaml(path: str | Path) -> dict:
    with Path(path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


class ChessExecutor(Node):
    def __init__(self, *, routes_path: str | Path | None = None):
        super().__init__("simplify_chess_executor")
        self.home = load_yaml(config_path("home.yaml"))
        self.safety = load_yaml(config_path("safety.yaml"))
        self.gripper_config = load_yaml(config_path("gripper.yaml"))
        self.db = RouteDatabase(routes_path or default_routes_path(),
                                self.home["home_joints"], self.home["home_tolerance_rad"])
        self.executor = TrajectoryExecutor(self, self.safety, self._log)
        self.gripper = GripperController(self, self.safety, self.gripper_config, self._log)
        self.log_path = Path(self.safety["log_path"])
        self.gripper_state = "UNKNOWN"

    def _log(self, *, square, trajectory_index, target, actual, label, success,
             gripper_state=None) -> None:
        event = {"timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "square": square,
                 "trajectory_index": trajectory_index, "target_arm1_arm5": target,
                 "actual_arm1_arm5": actual, "gripper_state": gripper_state or self.gripper_state,
                 "step": label, "success": success}
        self.get_logger().info(json.dumps(event, ensure_ascii=False))
        try:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=False) + "\n")
        except OSError as exc:
            self.get_logger().error(f"cannot write execution log: {exc}")

    def home_robot(self) -> None:
        self.executor.execute_waypoint(self.home["home_joints"], square="HOME", index=0,
                                       label="HOME confirmed")

    def set_gripper(self, state: str, square: str) -> None:
        self.gripper.set(state, square)
        self.gripper_state = state

    def stop(self) -> None:
        self.executor.stop()

    def test_square(self, square: str, *, reverse_only: bool = False,
                    allow_unvalidated: bool = False) -> None:
        route = self.db.test_route(square) if allow_unvalidated else self.db.executable_route(square)
        if reverse_only:
            # The operator is expected to have the arm at the square endpoint.
            # Do not silently replay outbound motion when explicitly testing return.
            self.executor.execute_route(list(reversed(route)), square=square,
                                        label="square -> HOME reverse test")
            return
        self.home_robot()
        self.executor.execute_route(route, square=square, label="HOME -> square test")
        # A return route must always begin from the square endpoint, never from HOME.
        self.executor.execute_route(list(reversed(route)), square=square,
                                    label="square -> HOME reverse test")

    def move(self, source: str, target: str) -> None:
        # Lookup first so missing/disabled routes fail before the robot starts moving.
        source_route = self.db.executable_route(source)
        target_route = self.db.executable_route(target)
        print(f"MOVE {source} -> {target}")
        print("[1] HOME confirmed")
        self.home_robot()
        print("[2] gripper PRE_CLOSE")
        self.set_gripper("PRE_CLOSE", source)
        print(f"[3] execute HOME -> {source}")
        self.executor.execute_route(source_route, square=source, label="HOME -> PICK")
        print("[4] gripper CLOSE")
        self.set_gripper("CLOSE", source)
        print(f"[5] execute {source} -> HOME")
        self.executor.execute_route(list(reversed(source_route)), square=source, label="PICK -> HOME")
        print(f"[6] execute HOME -> {target}")
        self.executor.execute_route(target_route, square=target, label="HOME -> DROP")
        print("[7] gripper PRE_CLOSE / RELEASE")
        self.set_gripper("PRE_CLOSE", target)
        print(f"[8] execute {target} -> HOME")
        self.executor.execute_route(list(reversed(target_route)), square=target, label="DROP -> HOME")
        print("MOVE COMPLETE")

    def status(self) -> str:
        validated = [square for square in self.db.data["routes"]
                     if self.db.get(square).status == "VALIDATED" and self.db.get(square).validated]
        return f"routes validated: {len(validated)}/64; current arm: {self.executor.actual_arm()}"

    def list_routes(self) -> list[str]:
        return [f"{square}: {self.db.get(square).status} ({len(self.db.get(square).route)} waypoints)"
                for square in self.db.data["routes"]]
