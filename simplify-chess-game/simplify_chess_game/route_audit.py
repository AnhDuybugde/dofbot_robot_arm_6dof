"""Offline route database audit; never contacts ROS or a motion planner."""

from __future__ import annotations

import argparse

from .chess_executor import config_path, default_routes_path, load_yaml
from .route_database import RouteDatabase


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit calibrated chess routes without moving the robot")
    parser.add_argument("--routes", default=str(default_routes_path()))
    args = parser.parse_args()
    home = load_yaml(config_path("home.yaml"))
    db = RouteDatabase(args.routes, home["home_joints"], home["home_tolerance_rad"])
    failures = db.validate_all()
    populated = sum(bool(db.get(square).route) for square in db.data["routes"])
    validated = sum(db.get(square).status == "VALIDATED" and db.get(square).validated
                    for square in db.data["routes"])
    print(f"routes populated: {populated}/64; validated: {validated}/64")
    if failures:
        for failure in failures:
            print(f"ERROR: {failure}")
        return 1
    print("route database shape audit: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
