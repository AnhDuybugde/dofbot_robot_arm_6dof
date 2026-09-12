"""Publish a fixed 8x8 board for the simplify chess RViz view."""
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import rclpy

BOARD_ORIGIN = (0.1105, -0.0915, 0.005)
SQUARE_SIZE = 0.026
BOARD_THICKNESS = 0.004
PIECES = {
    "p": (0.023, 0.015), "r": (0.027, 0.016), "n": (0.031, 0.017),
    "b": (0.035, 0.017), "q": (0.041, 0.018), "k": (0.047, 0.018),
}


class BoardVisualizer(Node):
    def __init__(self):
        super().__init__("simplify_chess_board_visualizer")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, "/chess/visual", qos)
        self.publish_board()

    def publish_board(self):
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
                light = (rank + file_index) % 2 == 0
                marker.color.r = 0.85 if light else 0.18
                marker.color.g = 0.72 if light else 0.22
                marker.color.b = 0.48 if light else 0.12
                marker.color.a = 1.0
                markers.markers.append(marker)
        # Initial chess position: ranks 1/2 are white, ranks 7/8 black.
        back = ["r", "n", "b", "q", "k", "b", "n", "r"]
        pieces = [(file_index, 0, piece, True) for file_index, piece in enumerate(back)]
        pieces += [(file_index, 1, "p", True) for file_index in range(8)]
        pieces += [(file_index, 6, "p", False) for file_index in range(8)]
        pieces += [(file_index, 7, piece, False) for file_index, piece in enumerate(back)]
        for index, (file_index, rank, kind, white) in enumerate(pieces):
            height, diameter = PIECES[kind]
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.ns = "simplify_chess_pieces"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = BOARD_ORIGIN[0] + rank * SQUARE_SIZE
            marker.pose.position.y = BOARD_ORIGIN[1] + file_index * SQUARE_SIZE
            marker.pose.position.z = BOARD_ORIGIN[2] + height / 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = height
            if white:
                marker.color.r, marker.color.g, marker.color.b = 0.92, 0.92, 0.88
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.08, 0.08, 0.08
            marker.color.a = 1.0
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
