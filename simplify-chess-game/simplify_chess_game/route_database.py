"""Editable YAML route database.  It deliberately knows nothing about XYZ or IK."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

SQUARES = tuple(f"{file}{rank}" for rank in "12345678" for file in "abcdefgh")


class RouteError(RuntimeError):
    pass


@dataclass(frozen=True)
class SquareRoute:
    square: str
    status: str
    validated: bool
    route: list[list[float]]
    notes: str


class RouteDatabase:
    def __init__(self, path: str | Path, home_joints: list[float], home_tolerance: float = 0.08):
        self.path = Path(path)
        self.home_joints = [float(x) for x in home_joints]
        self.home_tolerance = float(home_tolerance)
        self.data: dict[str, Any] = {}
        self.reload()

    def reload(self) -> None:
        with self.path.open(encoding="utf-8") as stream:
            self.data = yaml.safe_load(stream) or {}
        routes = self.data.setdefault("routes", {})
        for square in SQUARES:
            routes.setdefault(square, {"status": "UNCALIBRATED", "validated": False,
                                      "route": [], "notes": ""})

    def save(self) -> None:
        with self.path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(self.data, stream, sort_keys=False, allow_unicode=True)

    @staticmethod
    def _check_square(square: str) -> str:
        square = square.lower()
        if square not in SQUARES:
            raise RouteError(f"invalid square {square!r}; expected a1..h8")
        return square

    def get(self, square: str) -> SquareRoute:
        square = self._check_square(square)
        raw = self.data["routes"][square]
        route = [[float(q) for q in point] for point in raw.get("route", [])]
        for index, point in enumerate(route):
            if len(point) != 5:
                raise RouteError(f"{square}: waypoint {index} must contain arm1..arm5")
        return SquareRoute(square, str(raw.get("status", "UNCALIBRATED")),
                           bool(raw.get("validated", False)), route,
                           str(raw.get("notes", "")))

    def executable_route(self, square: str) -> list[list[float]]:
        entry = self.get(square)
        if entry.status != "VALIDATED" or not entry.validated:
            raise RouteError(f"square {entry.square} has no validated route "
                             f"(status={entry.status})")
        self._validate_shape_and_home(entry)
        return entry.route

    def test_route(self, square: str) -> list[list[float]]:
        entry = self.get(square)
        self._validate_shape_and_home(entry)
        return entry.route

    def _validate_shape_and_home(self, entry: SquareRoute) -> None:
        if len(entry.route) < 2:
            raise RouteError(f"square {entry.square} has no usable route; record HOME and a grip pose")
        first = entry.route[0]
        if any(abs(q - h) > self.home_tolerance for q, h in zip(first, self.home_joints)):
            raise RouteError(f"{entry.square}: first waypoint is not HOME; route rejected")

    def replace_route(self, square: str, route: list[list[float]], notes: str | None = None) -> None:
        square = self._check_square(square)
        cleaned = [[round(float(q), 6) for q in point] for point in route]
        for point in cleaned:
            if len(point) != 5:
                raise RouteError("every recorded waypoint must contain exactly 5 arm joints")
        raw = self.data["routes"][square]
        raw["route"] = cleaned
        raw["status"] = "UNCALIBRATED"
        raw["validated"] = False
        if notes is not None:
            raw["notes"] = notes

    def mark_validated(self, square: str, validated: bool, notes: str | None = None) -> None:
        square = self._check_square(square)
        entry = self.get(square)
        if validated:
            self._validate_shape_and_home(entry)
        raw = self.data["routes"][square]
        raw["status"] = "VALIDATED" if validated else "UNCALIBRATED"
        raw["validated"] = bool(validated)
        if notes is not None:
            raw["notes"] = notes

    def discard_route(self) -> SquareRoute:
        """Reserved capture destination, validated with the identical route rules."""
        raw = self.data.get("discard_route", {})
        entry = SquareRoute("DISCARD", str(raw.get("status", "UNCALIBRATED")),
                            bool(raw.get("validated", False)),
                            [[float(q) for q in point] for point in raw.get("route", [])],
                            str(raw.get("notes", "")))
        if entry.status != "VALIDATED" or not entry.validated:
            raise RouteError("DISCARD_ROUTE has no validated route")
        self._validate_shape_and_home(entry)
        return entry
