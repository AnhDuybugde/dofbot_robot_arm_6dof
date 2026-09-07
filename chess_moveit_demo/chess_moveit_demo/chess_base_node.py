"""Chế độ base không MoveIt: chạy luật cờ, Stockfish và visual RViz.

Node này cố ý không import pymoveit2 hay moveit_msgs.  Nó là mốc kiểm thử cho
logic /chess/move -> python-chess -> /chess/move_done trước khi nối lại robot.
"""

import json

import chess
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .chess_utils import BOARD_Z, PIECE_SPECS, square_to_xy


FRAME_ID = "world"


class ChessBaseNode(Node):
    def __init__(self):
        super().__init__("chess_base_node")
        self.board = chess.Board()
        self.visual_pub = self.create_publisher(
            MarkerArray,
            "/chess/visual",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.done_pub = self.create_publisher(String, "/chess/move_done", 10)
        self.move_sub = self.create_subscription(String, "/chess/move", self.on_move, 10)
        self._publish_position()
        self.get_logger().info(
            "Chess base sẵn sàng: chỉ chạy python-chess + Stockfish + RViz, không MoveIt."
        )

    def _square_marker(self, square: int) -> Marker:
        x, y = square_to_xy(chess.square_name(square))
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.ns = "chessboard"
        marker.id = square
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y = x, y
        marker.pose.position.z = BOARD_Z - 0.004
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.027
        marker.scale.z = 0.006
        light = (chess.square_file(square) + chess.square_rank(square)) % 2 == 1
        marker.color.r = 0.93 if light else 0.20
        marker.color.g = 0.78 if light else 0.12
        marker.color.b = 0.54 if light else 0.07
        marker.color.a = 1.0
        return marker

    def _piece_marker(self, square: int, piece: chess.Piece) -> Marker:
        x, y = square_to_xy(chess.square_name(square))
        spec = PIECE_SPECS[piece.symbol().lower()]
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.ns = "chess_pieces"
        # Marker ID cố định theo ô: mỗi lần publish là snapshot trọn vẹn.
        marker.id = square
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y = x, y
        marker.pose.position.z = BOARD_Z + spec.pickup_height / 2
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.021
        marker.scale.z = spec.pickup_height
        if piece.color:
            marker.color.r, marker.color.g, marker.color.b = 0.96, 0.96, 0.88
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.08, 0.10, 0.13
        marker.color.a = 1.0
        return marker

    def _publish_position(self):
        markers = [self._square_marker(square) for square in chess.SQUARES]
        # Xoá sạch marker quân cũ trước; nhờ vậy nước ăn quân không để lại bóng.
        clear = Marker()
        clear.header.frame_id = FRAME_ID
        clear.ns = "chess_pieces"
        clear.action = Marker.DELETEALL
        markers.append(clear)
        markers.extend(
            self._piece_marker(square, piece)
            for square, piece in self.board.piece_map().items()
        )
        self.visual_pub.publish(MarkerArray(markers=markers))

    def on_move(self, msg: String):
        try:
            payload = json.loads(msg.data)
            uci = payload["uci"]
            move = chess.Move.from_uci(uci)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Payload nước cờ không hợp lệ: {exc}")
            return
        if move not in self.board.legal_moves:
            self.get_logger().error(f"Từ chối nước đi không hợp lệ theo python-chess: {uci}")
            return
        self.board.push(move)
        self._publish_position()
        done = String()
        done.data = uci
        self.done_pub.publish(done)
        self.get_logger().info(f"Đã mô phỏng nước {uci}; gửi ACK để Stockfish đi tiếp.")


def main():
    rclpy.init()
    node = ChessBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

