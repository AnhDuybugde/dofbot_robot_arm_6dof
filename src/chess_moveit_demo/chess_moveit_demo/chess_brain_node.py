"""Node 'bộ não': KHÔNG đọc camera thật. Để Stockfish tự chơi cả 2 bên (self-play),
publish từng nước đi (dạng JSON) lên topic /chess/move để pick_place_node thực thi
trên robot mô phỏng trong RViz.

Cài đặt:
    pip install chess
    sudo apt install stockfish        # hoặc chỉnh STOCKFISH_PATH trỏ tới binary bạn có
"""

import json
import time

import chess
import chess.engine
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .chess_utils import ACK_TIMEOUT_SEC

STOCKFISH_PATH = "/usr/games/stockfish"   # chạy `which stockfish` để lấy đường dẫn đúng máy bạn
MOVE_TIME_LIMIT = 0.3                      # giây suy nghĩ mỗi nước, tăng nếu muốn nước đi "khôn" hơn


class ChessBrainNode(Node):
    def __init__(self):
        super().__init__("chess_brain_node")
        self.board = chess.Board()
        self.engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)

        self.move_pub = self.create_publisher(String, "/chess/move", 10)
        # TODO-1: chờ tín hiệu READY từ pick_place (latch transient-local),
        # thay cơ chế timer 12s cố định trong launch.
        self.system_ready = False
        self.ready_sub = self.create_subscription(
            String,
            "/chess/system_ready",
            self.on_system_ready,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        # pick_place_node báo lại khi robot đã thực thi xong nước đi, để brain đi tiếp
        self.ack_sub = self.create_subscription(
            String, "/chess/move_done", self.on_move_done, 10
        )
        # NACK từ pick_place: lỗi thực thi phải dừng game tường minh thay vì
        # treo ở waiting_for_ack vô hạn (không ACK nào bao giờ tới).
        self.fail_sub = self.create_subscription(
            String, "/chess/move_failed", self.on_move_failed, 10
        )
        self.last_pub_time = 0.0
        # Watchdog: lưới an toàn cuối cho trường hợp không ACK lẫn không NACK
        # (vd. worker thread chết bất thường). Ngưỡng 600s đủ rộng cho mọi nước
        # plan/execute bình thường.
        self.watchdog = self.create_timer(5.0, self._watchdog)
        self.waiting_for_ack = False
        self.timer = None
        self.game_running = False
        self.inflight_uci = None
        self.inflight_move = None
        # Fix 9: board của brain chỉ commit SAU ACK (không push khi quyết định),
        # nên board luôn = trạng thái executor đã xác nhận xong. command_id +
        # fen thay cho bare-UCI để ACK/NACK không thể khớp nhầm nước cũ.
        self.inflight_cmd = None
        self._next_command_id = 1
        # Khóa restart sau lỗi chưa phục hồi: executor có thể đang giữ quân
        # trên gripper/scene sai; cho start lại sẽ chạy tiếp trên scene sai.
        self.locked_after_failure = False
        self.move_count = 0
        self.start_srv = self.create_service(Trigger, "/chess/start", self.start_game)
        self.get_logger().info(
            "Chess brain sẵn sàng. Bàn cờ đang chờ: ros2 service call /chess/start std_srvs/srv/Trigger '{}'"
        )

    def on_system_ready(self, msg: String):
        if msg.data.strip().upper() == "READY" and not self.system_ready:
            self.system_ready = True
            self.get_logger().info("[READY] hạ tầng pick-place sẵn sàng, brain được phép đi.")

    def start_game(self, request, response):
        if self.game_running:
            response.success = False
            response.message = "Ván cờ đã chạy."
            return response
        if self.locked_after_failure:
            response.success = False
            response.message = (
                "Game đang bị KHÓA sau lỗi chưa phục hồi (scene/executor có thể "
                "sai). Restart launch rồi mới start ván mới.")
            return response
        if not self.system_ready:
            response.success = False
            response.message = (
                "Hạ tầng chưa READY (scene/planner/controller/joint_states). "
                "Chờ pick_place báo READY rồi start lại.")
            return response
        self.game_running = True
        # Timer one-shot: tuyệt đối không polling/phát lại nước cờ theo chu kỳ.
        # Nước kế tiếp chỉ được hẹn sau ACK khớp đúng UCI vừa gửi.
        self._schedule_next_tick(0.5)
        response.success = True
        response.message = "Bắt đầu self-play Stockfish."
        self.get_logger().info(response.message)
        return response

    def _schedule_next_tick(self, delay_sec: float):
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(delay_sec, self.tick)

    def tick(self):
        # create_timer là periodic theo mặc định; biến nó thành one-shot ngay
        # đầu callback để không thể gọi tick lần nữa trước ACK.
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        if not self.game_running or self.waiting_for_ack:
            return
        if not self.system_ready:
            self.get_logger().warning(
                "[NOT-READY] brain chờ hạ tầng READY, chưa đi nước tiếp.",
                throttle_duration_sec=5.0,
            )
            self._schedule_next_tick(1.0)
            return
        if self.board.is_game_over():
            self.get_logger().info(f"Ván kết thúc: {self.board.result()}")
            self.game_running = False
            return

        try:
            result = self.engine.play(self.board, chess.engine.Limit(time=MOVE_TIME_LIMIT))
            if result.move is None or result.move not in self.board.legal_moves:
                raise RuntimeError("engine returned no legal move")
        except Exception as exc:
            self._stop_game(f"engine failure: {exc}")
            return
        move = result.move
        piece = self.board.piece_at(move.from_square)

        # Trắng là người điều khiển robot; Đen là đối thủ ảo. Cả hai vẫn đi qua
        # pick_place_node để node đó là nguồn chân lý duy nhất cho board/visual/
        # planning scene, nhưng chỉ "robot" mới gửi trajectory tới MoveIt.
        execution = "robot" if piece.color == chess.WHITE else "virtual"
        cmd = self._next_command_id
        self._next_command_id += 1
        payload = {
            "command_id": cmd,
            "board_fen": self.board.fen(),
            "uci": move.uci(),
            "piece_type": piece.symbol().lower(),
            "execution": execution,
            "capture": self.board.is_capture(move),
            "castling": self.board.is_castling(move),
            "en_passant": self.board.is_en_passant(move),
            # Gợi ý hiển thị; executor SUY promotion từ nước hợp lệ, không tin
            # trường này (xem _execute_move/_do_virtual_move).
            "promotion": chess.piece_symbol(move.promotion) if move.promotion else None,
        }

        # Fix 9: KHÔNG push board ở đây. Board chỉ commit trong on_move_done khi
        # ACK khớp command_id + uci — executor và brain không thể lệch nhau.
        self.inflight_move = move
        self.inflight_uci = move.uci()
        self.inflight_cmd = cmd
        msg = String()
        msg.data = json.dumps(payload)
        self.waiting_for_ack = True
        self.last_pub_time = time.monotonic()
        self.move_pub.publish(msg)
        self.move_count += 1
        # Log gọn terminal launch: thành công 1 dòng ngắn, lỗi mới chi tiết.
        self.get_logger().info(
            f"[OK] nước {self.move_count}: {move.uci()} ({execution}) [cmd={cmd}]"
        )

    @staticmethod
    def _parse_ack(data: str):
        """Tách '<cmd>:<uci>[: <reason>]'; dữ liệu sai format trả cmd=0."""
        parts = data.split(":", 2)
        if len(parts) >= 2 and parts[0].strip().lstrip("-").isdigit():
            cmd = int(parts[0].strip())
            rest = parts[1].strip()
            reason = parts[2].strip() if len(parts) > 2 else ""
            return cmd, rest, reason
        return 0, data.strip(), ""

    def on_move_done(self, msg: String):
        if not self.waiting_for_ack:
            self.get_logger().warning(f"[FAIL] ACK dư thừa (không chờ): {msg.data}")
            return
        cmd, uci, _ = self._parse_ack(msg.data)
        if uci != self.inflight_uci or cmd != self.inflight_cmd:
            self.get_logger().warning(
                f"[FAIL] ACK sai nước: nhận cmd={cmd} {uci}, đang chờ "
                f"cmd={self.inflight_cmd} {self.inflight_uci}"
            )
            return
        # Commit board SAU ACK khớp — đây mới là lúc nước đi chắc chắn xong.
        self.board.push(self.inflight_move)
        self.waiting_for_ack = False
        self.inflight_uci = None
        self.inflight_move = None
        self.inflight_cmd = None
        # Chỉ ACK thành công mới dẫn đến đúng một nước kế tiếp.
        self._schedule_next_tick(0.3)

    def on_move_failed(self, msg: String):
        """NACK từ pick_place (format '<cmd>:<uci>: <lý do>'). Dừng game tường minh."""
        cmd, uci, _ = self._parse_ack(msg.data)
        if not self.waiting_for_ack:
            self.get_logger().warning(f"[FAIL] NACK dư thừa (không chờ): {msg.data}")
            return
        if uci not in (self.inflight_uci, "?") or cmd != self.inflight_cmd:
            self.get_logger().warning(
                f"[FAIL] NACK sai nước: nhận cmd={cmd} {msg.data}, đang chờ "
                f"cmd={self.inflight_cmd} {self.inflight_uci}"
            )
            return
        self._stop_game(f"nước {self.inflight_uci} thất bại phía robot: {msg.data}")

    def _stop_game(self, reason: str):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        self.game_running = False
        self.waiting_for_ack = False
        self.inflight_uci = None
        self.inflight_move = None
        self.inflight_cmd = None
        # Khóa restart: board brain chưa commit nước lỗi, executor có thể kẹt
        # scene — start lại mù sẽ chạy tiếp trên trạng thái sai (Fix 9).
        self.locked_after_failure = True
        self.get_logger().error(
            f"[FAIL] Game DỪNG + KHÓA restart: {reason}. "
            f"Cần restart launch để chơi ván mới."
        )

    def _watchdog(self):
        if self.game_running and self.waiting_for_ack and self.last_pub_time:
            stalled = time.monotonic() - self.last_pub_time
            if stalled > ACK_TIMEOUT_SEC:
                self._stop_game(
                    f"treo {stalled:.0f}s không ACK/NACK cho {self.inflight_uci}"
                )

    def destroy_node(self):
        self.engine.quit()
        super().destroy_node()


def main():
    rclpy.init()
    node = ChessBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
