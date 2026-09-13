"""Stateful 8x8 board for the simplify chess RViz view.

Tracks which piece stands on which square plus the piece currently carried
by the gripper. ChessExecutor publishes pick/drop/reset events on
/chess/piece_events; this node republishes the full MarkerArray at 5 Hz so
a carried piece visibly follows the arm (via the Gripping_point_Link TF)
instead of staying frozen on its start square.
"""
from __future__ import annotations

import json

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
import tf2_ros

BOARD_ORIGIN = (0.1105, -0.0915, 0.005)
SQUARE_SIZE = 0.026
BOARD_THICKNESS = 0.004
PIECES = {
    "p": (0.023, 0.015), "r": (0.027, 0.016), "n": (0.031, 0.017),
    "b": (0.035, 0.017), "q": (0.041, 0.018), "k": (0.047, 0.018),
}
FILES = "abcdefgh"
CARRIED_ID = 64
HOVER_HEIGHT = 0.06


def initial_setup() -> dict[str, tuple[str, bool]]:
    back = ["r", "n", "b", "q", "k", "b", "n", "r"]
    setup: dict[str, tuple[str, bool]] = {}
    for file_index, piece in enumerate(back):
        setup[f"{FILES[file_index]}1"] = (piece, True)
        setup[f"{FILES[file_index]}2"] = ("p", True)
        setup[f"{FILES[file_index]}7"] = ("p", False)
        setup[f"{FILES[file_index]}8"] = (piece, False)
    return setup


def square_xy(square: str) -> tuple[float, float]:
    file_index = FILES.index(square[0])
    rank = int(square[1]) - 1
    return (BOARD_ORIGIN[0] + rank * SQUARE_SIZE,
            BOARD_ORIGIN[1] + file_index * SQUARE_SIZE)


def square_index(square: str) -> int:
    return (int(square[1]) - 1) * 8 + FILES.index(square[0])


class BoardVisualizer(Node):
    def __init__(self):
        super().__init__("simplify_chess_board_visualizer")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, "/chess/visual", qos)
        self.create_subscription(String, "/chess/piece_events", self.on_piece_event, 10)
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.squares: dict[str, tuple[str, bool]] = initial_setup()
        self.carried: dict | None = None
        self.create_timer(0.2, self.republish)

    def on_piece_event(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn(f"ignoring malformed piece event: {message.data!r}")
            return
        kind = event.get("event")
        square = str(event.get("square", "-")).lower()
        if kind == "reset":
            self.squares = initial_setup()
            self.carried = None
        elif kind == "pick":
            if self.carried is not None:
                self.get_logger().warn("pick while already carrying; dropping previous")
                self.carried = None
            piece = self.squares.pop(square, None)
            if piece is None:
                self.get_logger().warn(f"pick on empty square {square}; nothing carried")
                return
            self.carried = {"kind": piece[0], "white": piece[1], "from": square}
        elif kind == "drop":
            if self.carried is None:
                self.get_logger().warn(f"drop on {square} with empty gripper; ignored")
                return
            captured = self.squares.get(square)
            if captured is not None:
                self.get_logger().info(f"capture on {square}: {captured} removed")
            self.squares[square] = (self.carried["kind"], self.carried["white"])
            self.carried = None
        elif kind == "move":
            # Opponent (black) move with no arm motion: teleport the piece so
            # the display stays in sync with the python-chess game state.
            source = str(event.get("from", "-")).lower()
            target = str(event.get("to", "-")).lower()
            piece = self.squares.pop(source, None)
            if piece is None:
                self.get_logger().warn(f"move from empty square {source}; ignored")
                return
            captured = self.squares.get(target)
            if captured is not None:
                self.get_logger().info(f"capture on {target}: {captured} removed")
            self.squares[target] = piece
        else:
            self.get_logger().warn(f"unknown piece event: {message.data!r}")

    def gripper_xy(self) -> tuple[float, float, float] | None:
        try:
            transform = self.tf.lookup_transform("base_link", "Gripping_point_Link", Time())
        except Exception:
            return None
        position = transform.transform.translation
        return (position.x, position.y, position.z)

    def piece_marker(self, marker_id: int, kind: str, white: bool,
                     x: float, y: float, z: float) -> Marker:
        height, diameter = PIECES[kind]
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.ns = "simplify_chess_pieces"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = height
        if white:
            marker.color.r, marker.color.g, marker.color.b = 0.92, 0.92, 0.88
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.08, 0.08, 0.08
        marker.color.a = 1.0
        return marker

    def republish(self):
        markers = MarkerArray()
        for rank in range(8):
            for file_index in range(8):
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.ns = "simplify_chess_board"
                marker.id = rank * 8 + file_index
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = BOARD_ORIGIN[0] + rank * SQUARE_SIZE
                marker.pose.position.y = BOARD_ORIGIN[1] + file_index * SQUARE_SIZE
                marker.pose.position.z = BOARD_ORIGIN[2] - BOARD_THICKNESS / 2.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = SQUARE_SIZE * 0.96
                marker.scale.y = SQUARE_SIZE * 0.96
                marker.scale.z = BOARD_THICKNESS
                # Chuẩn cờ: a1 là ô tối. (file+rank)%2==1 mới là ô sáng,
                # khớp chess_base_node/pick_place_node dùng python-chess.
                light = (rank + file_index) % 2 == 1
                marker.color.r = 0.85 if light else 0.18
                marker.color.g = 0.72 if light else 0.22
                marker.color.b = 0.48 if light else 0.12
                marker.color.a = 1.0
                markers.markers.append(marker)
        occupied = set()
        for square, (kind, white) in self.squares.items():
            x, y = square_xy(square)
            height, _ = PIECES[kind]
            markers.markers.append(self.piece_marker(
                square_index(square), kind, white, x, y, BOARD_ORIGIN[2] + height / 2.0))
            occupied.add(square_index(square))
        # Empty squares must explicitly DELETE, otherwise RViz keeps the
        # stale piece marker after a pick.
        for rank in range(8):
            for file_index in range(8):
                index = rank * 8 + file_index
                if index in occupied:
                    continue
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.ns = "simplify_chess_pieces"
                marker.id = index
                marker.action = Marker.DELETE
                markers.markers.append(marker)
        if self.carried is not None:
            kind, white = self.carried["kind"], self.carried["white"]
            height, _ = PIECES[kind]
            gripper = self.gripper_xy()
            if gripper is not None:
                x, y, z = gripper[0], gripper[1], gripper[2] - height / 2.0 - 0.005
            else:
                # No TF (visualizer alone): hover above the pick square.
                x, y = square_xy(self.carried["from"])
                z = BOARD_ORIGIN[2] + HOVER_HEIGHT
            markers.markers.append(self.piece_marker(CARRIED_ID, kind, white, x, y, z))
        else:
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.ns = "simplify_chess_pieces"
            marker.id = CARRIED_ID
            marker.action = Marker.DELETE
            markers.markers.append(marker)
        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = BoardVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
